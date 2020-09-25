/*****************************************************************************
* Hand-off management
 *handoff.cpp : LTE System Level Simulator
 *****************************************************************************
 * Copyright (C) 2010 LTE System Level Simulator project
 * Authors: "Pooya Monajemi" <pmonajemi@ucla.edu>,
 *          "Shaunak Joshi" <sjoshi@ee.ucla.edu>,
 *          "Jianwen Chen" <jwchen@ee.ucla.edu>,
 *          "Julio Navarro Lara" <julionl@ucla.edu>
 *
 *
 *****************************************************************************/
#include <math.h>
#include <stdlib.h>
#include "Predictive_Handoff.h"
#include "Utils.h"

//-------------------------------------------------------------
Tile::Tile(Simulator * sim, int id,vector<int> QoSList):ID(id),QoS_BS(QoSList)
{
	// initialize Tile 
}
//-------------------------------------------------------------
Position::Position(double X, double Y, Road* road, double prob)
{
	this->X = X;
	this->Y = Y;
	this->road = road;
	this->prob = prob;
}
//-------------------------------------------------------------
Predictive_Handoff::Predictive_Handoff(Mobile_Station *ms, string cfgstr)
{
	Sim = ms->Sim;
	MS = ms;

	readcfg(cfgstr);


	//maxDepth = 2;
	//samplingPeriod = 10; 
	//BSMinPresence = 0.9;
}
//-------------------------------------------------------------
void Predictive_Handoff::readcfg(string cfgstr)
{
	try{
		maxDepth		=	readField<int>(cfgstr,"maxDepth",1);
		samplingPeriod	=	readField<double>(cfgstr,"samplingPeriod",1);
		BSMinPresence	=	readField<double>(cfgstr,"QoSPorb",1);
	}
	catch(string str){
		throw str;
	}
}
//-------------------------------------------------------------
vector<Position> Predictive_Handoff::prediction(int depth, Position* pos)
{
	vector<Position> paths;
	vector<Position> newPaths;

	//We repeat the recursive method until 'depth' reaches 0
	if(depth > 0){

		iteration = 0;

		PROCESS_MEMORY_COUNTERS_EX pmc;
		int result;
		int ma, mb;
		result = GetProcessMemoryInfo(GetCurrentProcess(),(PROCESS_MEMORY_COUNTERS*)&pmc,sizeof( pmc ));
		ma = pmc.PrivateUsage;

		//Prediction of the different positions of the MS after the sampling period
		vector<Position> positions = movement(pos,samplingPeriod);

		PROCESS_MEMORY_COUNTERS_EX pmc2;
	result = GetProcessMemoryInfo(GetCurrentProcess(),(PROCESS_MEMORY_COUNTERS*)&pmc2,sizeof( pmc2 ));
	//printf( "M: %i\n", (pmc2.PrivateUsage-ma));

		//For each position we need to do another prediction
		for(int i=0; i<(int)positions.size(); i++)
		{
			//We call the method recursively changing the depth value and from the new position
			newPaths = prediction(depth-1, &positions[i]);

			//Data processing of each final prediction
			for(int j=0; j<(int)newPaths.size(); j++)
			{
				vector<vector<int> > tempvector (Sim->numBS, vector<int>(maxDepth+1,0)); //Temporal vector

				//We need to add the information of each possible BS in this tile to the compiled information from each prediction
				for(int k=0; k<(int)pos->road->SelfTilePtr->QoS_BS.size(); k++)		
				{
					/*The matrix BSsequences has a summary of the number of jumps of each possible path
					The rows represent the BS index and the columns are the number of jumps. The maximum number of
					jumps cannot be greater than the current depth of our recursive method, so we use the depth
					as a reference for going through the columns of the matrix.
					The number in each position is the number of handoff paths we have with that number of jumps (column) and 
					starting with that BS (row), so we can later determine which is the best handoff path for each final position
					of the MS.
					*/

					//Going through the columns
					for(int l=0; l<depth; l++)
					{
						//Going through the rows
						for(int h=0; h<Sim->numBS; h++)
						{
							//If that BS index is in the current tile, we dont have any new jump and we clone
							//the number of paths with that amount of handoff jumps
							if(h==pos->road->SelfTilePtr->QoS_BS[k])
							{
								tempvector[h][l] += newPaths[j].BSsequences[h][l];
							}
							else
							{
								//In any other case, we have a new jump and we add the information to the next column and
								//to the row of the BS that we are exploring
								tempvector[pos->road->SelfTilePtr->QoS_BS[k]][l+1] += newPaths[j].BSsequences[h][l];
							}
						}
					}
				}
				
				//Storage of the matrix
				newPaths[j].BSsequences = tempvector;
			}

			//We include the new paths in the path array
			paths.insert(paths.end(),newPaths.begin(),newPaths.end());

			//If we are in the root of the tree (beginning), we need to take information of the BS appearance
			//in the next prediction step. We will use this information later in the BS update.
			if(depth == maxDepth)
			{
				for(int l=0; l<(int)positions[i].road->SelfTilePtr->QoS_BS.size(); l++)
				{
					nextBS[positions[i].road->SelfTilePtr->QoS_BS[l]] += positions[i].prob;
				}

			}
		}

		return paths; //Return of all the possible final positions
	}
	else
	{
		//If we are at the end of the predicition tree, we need to create the Position object
		Position* endPosition = new Position(pos->X,pos->Y,pos->road,pos->prob);
		vector<vector<int> > tempvector (Sim->numBS, vector<int>(maxDepth+1,0));

		//We have just routes with 0 jumps so far
		for(int k=0; k<(int)pos->road->SelfTilePtr->QoS_BS.size(); k++)
		{
			tempvector[pos->road->SelfTilePtr->QoS_BS[k]][0]++;
		}

		endPosition->BSsequences = tempvector;
		paths.push_back(*endPosition);
		delete endPosition;
		return paths;
	}
}
//-------------------------------------------------------------
vector<Position> Predictive_Handoff::movement(Position* currentPos, double ts)
{
	vector<Position> positions;
	vector<Position> newPositions;

	iteration++;
	
	//Extremes of the current road
	double x1 = currentPos->road->StrtEndPts[0];
	double y1 = currentPos->road->StrtEndPts[1];
	double x2 = currentPos->road->StrtEndPts[2];
	double y2 = currentPos->road->StrtEndPts[3];				
	
	double theta = atan((y1-y2)/(x1-x2)); //Angle of the road
	//It is going to be always between -pi/2 and pi/2

	//We need eventually to include a correction because of the atan ambiguity
	if(x1-x2 > 0)
		theta = theta + 3.14159;

	//Distance = speed * time
	double d = (currentPos->road->speed)*ts;

	//Trigonometry for finding the new position of the MS
	double X = currentPos->X + d*cos(theta);
	double Y = currentPos->Y + d*sin(theta);

	Position* pos;

	//We need to check if this position is outside the road or not
	if(X > max(x1,x2) || X < min(x1,x2) || Y > max(y1,y2) || Y < min(y1,y2))
	{
		// ------------Out of the Road-----------//
		// -------- Choose next Road depending on the given probabilities ------- //

		if (currentPos->road->nextRoads.size()<=0) // If there is no next Road i.e deadend of a Road, it will stop moving
		{
			X = x2;  //We stop at the end
			Y = y2;
			pos = new Position(X,Y,currentPos->road,currentPos->prob);
			positions.push_back((*pos));
			delete pos;
			return positions;
		}

		double p;
		double timeleft;
		Road* newRoad;

		//Going through all the possible next roads
		for (int i=0; i < (int) currentPos->road->nextRoads.size(); i++)
		{
			//The probability of this new position will be affected by the
			//prob. of the current position and the prob. of change
			//We suppose independent events
			p = currentPos->prob * currentPos->road->nextRoadProbs[i];
		
			//According to the distance from the current point to the end of the road, we need to
			//calculate the time left for the rest of the movement
			timeleft = sqrt(pow(X-x2,2)+pow(Y-y2,2))/((currentPos->road->speed));

			//With the new position (end of the current road) and new road we call recursively
			//the movement method again

			newRoad = currentPos->road->nextRoads[i];

			pos = new Position(newRoad->StrtEndPts[0],newRoad->StrtEndPts[1],newRoad,p);

			newPositions = movement(pos, timeleft);
			iteration--;

			//Finally, we take all the positions together
			positions.insert(positions.end(),newPositions.begin(),newPositions.end());

			delete pos;
		}
		return positions;

	}else{
		if(Sim->sim_stats->MS_stats[MS->Ind].VerboseMode && iteration>2){
			MS->Iterations.push_back(iteration);
		}
		//If we are in the same road, we just change the coordinates of position
		pos = new Position(X,Y,currentPos->road,currentPos->prob);
		positions.push_back((*pos));
		delete pos;
		return positions;
	}
}

//-------------------------------------------------------------
void Predictive_Handoff::update()
{
	vector<double> temp (Sim->numBS,0);
	nextBS = temp;
	MS->Iterations.clear();
	MS->HOflag = 1;

	//Current position of the MS, with (relative) probability = 1.0
	Position *pos = new Position(MS->X,MS->Y,MS->CurrentRoad,1.0);

	//Calculation of all the possible final positions
	vector<Position> paths = prediction(maxDepth,pos);

	//Vector for storing the probability of changing to a BS
	vector<double> BSprobs (Sim->numBS,0.0);

	//Going through all the final positions
	for(int i=0; i<(int)paths.size(); i++)
	{
		//Going through the columns of the "handoff matrix" in that route
		//We will stop in the first column we find with non-zero numbers (minimum number of jumps)
		for(int k=0; k<maxDepth+1; k++)
		{
			int counter = 0; //Counter for summing up all the element of a column
			//Going through the rows, but just through those which represent a BS presented in this tile
			//We do this for saving time
			for(int j=0; j<(int)MS->CurrentRoad->SelfTilePtr->QoS_BS.size(); j++)
			{

				if(MS->CurrentRoad->SelfTilePtr->QoS_BS[j]==MS->Connect_BS_ind)
				{
					if(paths[i].BSsequences[MS->Connect_BS_ind][k]!=0)
					{
						//If the BS checked is the same than the current BS and the number of paths is != 0,
						//we are going to stay in that BS for minimizing the energy
						BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]] = paths[i].prob;
						counter = -1;
						break;
					}else if(k<maxDepth){
						//If the number of paths is 0, we would need to take the number from the next column, because
						//we need a jump less for that BS due to we already are attached to it
						counter += paths[i].BSsequences[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]][k+1];
					}
				}
				else
				{
					//If we are not attached to that BS, we just increment the counter
					counter += paths[i].BSsequences[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]][k];
				}
			}

			//If counter == 0, we just go to the next column.
			//We are going to choose a path from the first column where counter == 0
			//All the next columns represent a higher number of jumps
			if(counter==-1)
				break;
			else if(counter!=0)
			{
				//We go again through all the BS for computing the probabilities using the total value in counter
				for(int j=0; j<(int)MS->CurrentRoad->SelfTilePtr->QoS_BS.size(); j++)
				{	
					//Remember that if we are in the current BS, we need to look to the next column for taking into account
					//that we dont need the first jump
					if(MS->CurrentRoad->SelfTilePtr->QoS_BS[j]==MS->Connect_BS_ind && k<maxDepth)
					{
						BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]] += 
							( ((double)paths[i].BSsequences[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]][k+1]) / (double)counter )*paths[i].prob;
					}
					else
					{
						BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]] += 
							( ((double)paths[i].BSsequences[MS->CurrentRoad->SelfTilePtr->QoS_BS[j]][k]) / (double)counter )*paths[i].prob;
					}
				}
				break; //We already have what we want, so we leave
			}
		}
	}

	delete pos;

	int mostlikely = -1;				//Most likely BS index
	double mostlikelyProb = 0.0;		//Most likely BS probability
	double BSPresence = BSMinPresence;  //Minimum presence of the BS in the next step for being useful

	for(int i=0; i<(int)MS->CurrentRoad->SelfTilePtr->QoS_BS.size(); i++)
	{
		//For each BS in this tile, if the probability of appearance of that BS in the future is higher than the
		//previous one's probability and the presence in next step is higher than our limit, we choose as far that BS
		//as the most likely
		if((BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[i]] > mostlikelyProb || 
			(BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[i]] = mostlikelyProb && MS->CurrentRoad->SelfTilePtr->QoS_BS[i]==MS->Connect_BS_ind)) 
			&& nextBS[MS->CurrentRoad->SelfTilePtr->QoS_BS[i]]>BSPresence)
		{
			mostlikely = MS->CurrentRoad->SelfTilePtr->QoS_BS[i];
			mostlikelyProb = BSprobs[MS->CurrentRoad->SelfTilePtr->QoS_BS[i]];
		}
		//If we already checked all the base station and we didnt find any result, we would need to low
		//our presence limit (by 0.1)
		if(i==(int)MS->CurrentRoad->SelfTilePtr->QoS_BS.size()-1 && mostlikely == -1 && BSPresence > 0)
		{
			i=0;
			BSPresence -= 0.1;
			if(BSPresence < 0)
				BSPresence = 0;
		}
	}

	if(mostlikely==-1)
	{
		//If we dont have any most likely BS and the current BS is reachable in this
		//tile, the best option is staying attached to it
		for(int i=0; i<(int)MS->CurrentRoad->SelfTilePtr->QoS_BS.size(); i++)
		{
			if(MS->CurrentRoad->SelfTilePtr->QoS_BS[i]==MS->Connect_BS_ind)
			{
				return;
			}
		}

		//If we need to change, we use the Instantaneous Pilot scheme
		MS->Connect_BS_ind = Sim->network->FindMaxPilotInst(MS->Ind);
	}
	else
	{
		//Connection to the most likely BS
		MS->Connect_BS_ind = mostlikely;
	}

	if(Sim->sim_stats->MS_stats[MS->Ind].VerboseMode){
		MS->BSprobs = BSprobs;
		MS->BSPresence = nextBS;
	}
}



//-------------------------------------------------------------
// Match using QoSList and check for existing Tiles
int matchTile(Simulator * Sim, vector <int> QoSList) // Return Matched Tile ID else -1
{
	RNC * rnc = Sim->network;

	int match = -1;
	for (int i=0; i<(int) rnc->Tiles.size();i++)
	{
		if (QoSList.size() == rnc->Tiles[i].QoS_BS.size())
		{
			if (equal (QoSList.begin(), QoSList.end(), rnc->Tiles[i].QoS_BS.begin()))
				match = i;
		}
	}
	return match;
}

//-------------------------------------------------------------
vector<double> LineCircleIntersect(double x1,double y1,double x2,double y2,double cx,double cy,double radius)
{
	vector<double> T;

	double a = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
	double b = 2*( (x1-cx)*(x2-x1) + (y1-cy)*(y2-y1)); //x2 + 2*Sim->BS[j].X*(x2-x1) - 2*x2*x2 + 2*y2 + 2*Sim->BS[j].Y*(y2-y1) - 2*y2*y2 ;
	double c = (x1-cx)*(x1-cx) + (y1-cy)*(y1-cy) - radius*radius;
			
	if ((b*b-4*a*c) > 0)
	{
		double t1 = (- b + sqrt(b*b-4*a*c))*0.5/a;
		double t2 = (- b - sqrt(b*b-4*a*c))*0.5/a;

		if (t1>0 && t2>0 && t1<1 && t2<1){ 
			T.push_back(t1); 
			T.push_back(t2); 
		}
		else if (t1>0 && t1<1){ 
			T.push_back(t1); 
		}
		else if (t2>0 && t2<1){
			T.push_back(t2); 
		}
	}

	return T;
}
//-------------------------------------------------------------
void CreateTiles(Simulator * Sim)
{

	int initial = Sim->numRoads;
	int numTiles = 0;				//	Number Of Tiles
	int i,j,k;

	vector <int> QoSList;
	vector <int> QoSList_1;
	vector <int> QoSList_2;

	RNC * rnc = Sim->network;

	VectorMap RoadSplits; // A map to keep track of each original road ID and the final (split) roads' IDs

	for (i=0;i<initial;i++)
	{
		//               Algorithm to search boundaries

		// Find the intersections between the line segment and circles around the base station
		vector <double> intrsctn_list;
		double x1 = Sim->Roads[i].StrtEndPts[0];
		double y1 = Sim->Roads[i].StrtEndPts[1];
		double x2 = Sim->Roads[i].StrtEndPts[2];
		double y2 = Sim->Roads[i].StrtEndPts[3];
		

		// Solving the quadratic formula : 
		//         (x1 + t(x2-x1) - cx)^2  +  (y1 + t(y2-y1) - cy)^2  = r^2
		for (j=0;j<Sim->numBS;j++)
		{
			double cx = Sim->BS[j].X;
			double cy = Sim->BS[j].Y;
			double radius = Sim->BS[j].radius;
			vector<double> T = LineCircleIntersect(x1,y1,x2,y2,cx,cy,radius);
			intrsctn_list.insert(intrsctn_list.end(),T.begin(),T.end());
		}

		// Sort the intrsctn_list
		sort(intrsctn_list.begin(),intrsctn_list.end());

		// Let the first t paired with start point retain the original Road ID but modify its start and endpoints
		// All pair except last will have only 1 nextRoad with p=1
		// Last t paired with end point will retain the nextRoad vector and their assoc p's of the original Road
		// All pair will retain the speed 
		// After sorting find QoS list for the midpoint of each pair of consecutive ts
		// Each pair will be assigned Tile looking at QoS list 
		for (k = intrsctn_list.size() ; k >= 0 ; k--)
		{
			Road temp;
			temp.speed = Sim->Roads[i].speed;
			temp.density = Sim->Roads[i].density;
			temp.StrtEndPts.resize(4);

			bool newRoad = false;

			if (intrsctn_list.size() == 0) 
			{   //Do Not touch
				newRoad = false;

				// Fill in the temp information for computing the middle of the road in the QOS step
				temp.StrtEndPts[0] = x1;
				temp.StrtEndPts[1] = y1;
				temp.StrtEndPts[2] = x1;
				temp.StrtEndPts[3] = y2;

				RoadSplits[Sim->Roads[i].ID].push_back(Sim->Roads[i].ID); // The original road ID
			}				
			else if(k==0)
			{
				// Shorten the original road segment to only reach the first intersection
				newRoad = false;

				Sim->Roads[i].StrtEndPts[2] = x1 + intrsctn_list[k]*(x2 - x1);
				Sim->Roads[i].StrtEndPts[3] = y1 + intrsctn_list[k]*(y2 - y1);
				Sim->Roads[i].nextRoadProbs.clear();
				Sim->Roads[i].nextRoads.clear();
				Sim->Roads[i].nextRoadIDs.clear();
				Sim->Roads[i].nextRoadProbs.push_back(1);
				Sim->Roads[i].nextRoads.push_back(&(Sim->Roads[Sim->numRoads-1]));
				Sim->Roads[i].nextRoadIDs.push_back(Sim->Roads[Sim->numRoads-1].ID);

				// Fill in the temp information for computing the middle of the road in the QOS step
				temp.StrtEndPts[0] = x1;
				temp.StrtEndPts[1] = y1;
				temp.StrtEndPts[2] = Sim->Roads[i].StrtEndPts[2];
				temp.StrtEndPts[3] = Sim->Roads[i].StrtEndPts[3];

				RoadSplits[Sim->Roads[i].ID].push_back(Sim->Roads[i].ID); // The original ID is still part of the original road
			}
			else if (k == intrsctn_list.size())
			{
				// The road segment between the last intersection and the end point. Transfer the 
				// next road transition properties

				newRoad = true;
				
				Sim->numRoads += 1; 
				temp.nextRoadIDs = Sim->Roads[i].nextRoadIDs;
				temp.nextRoadProbs = Sim->Roads[i].nextRoadProbs;
				temp.ID = Sim->numRoads;
				temp.StrtEndPts[0] = x1 + intrsctn_list[k-1]*(x2 - x1);
				temp.StrtEndPts[1] = y1 + intrsctn_list[k-1]*(y2 - y1);
				temp.StrtEndPts[2] = Sim->Roads[i].StrtEndPts[2];
				temp.StrtEndPts[3] = Sim->Roads[i].StrtEndPts[3];

				RoadSplits[Sim->Roads[i].ID].push_back(temp.ID); // A new split of the original road
			}
			else
			{
				// A road segment in the middle. 

				newRoad = true;
				Sim->numRoads += 1;
				temp.ID = Sim->numRoads;
				temp.StrtEndPts[0] = x1 + intrsctn_list[k-1]*(x2 - x1);
				temp.StrtEndPts[1] = y1 + intrsctn_list[k-1]*(y2 - y1);
				temp.StrtEndPts[2] = x1 + intrsctn_list[k]*(x2 - x1);
				temp.StrtEndPts[3] = y1 + intrsctn_list[k]*(y2 - y1);
				temp.nextRoadProbs.push_back(1);
				temp.nextRoads.push_back(&(Sim->Roads[Sim->numRoads-2]));
				temp.nextRoadIDs.push_back(Sim->Roads[Sim->numRoads-2].ID);

				RoadSplits[Sim->Roads[i].ID].push_back(temp.ID); // A new split of the original road
			}

			//  Assign New/Existing Tile 
			double midX = 0.5*(temp.StrtEndPts[0]+temp.StrtEndPts[2]);
			double midY = 0.5*(temp.StrtEndPts[1]+temp.StrtEndPts[3]);
			
			QoSList.clear();

			for (j=0;j<Sim->numBS;j++)
			{
				if (Distance(midX,midY,Sim->BS[j].X,Sim->BS[j].Y) < Sim->BS[j].radius) // d is the rad
					QoSList.push_back(Sim->BS[j].ID);
			}

			// Check if QoS list matches with existing Tile
			int match = matchTile(Sim,QoSList);
			Tile * Tileptr;
			if( match >= 0)
			{
				Tileptr = &(rnc->Tiles[match]);
			}
			else
			{
				// If no match found : NEW Tile!
				Tile *newTile = new Tile(Sim, numTiles, QoSList);
				rnc->Tiles.push_back(*newTile);
				Tileptr = &(rnc->Tiles[numTiles]);
				numTiles++;
			}


			if(newRoad){
				temp.Ind = Sim->Roads.size();
				temp.TileID = Tileptr->ID;
				Sim->Roads.push_back(temp);
			}
			else
				Sim->Roads[i].TileID = Tileptr->ID;
		}
		
	}
	ResetNextRoadPtrs(Sim);
	SetRoadTilePtrs(Sim, rnc);
	ResetMSRoads(Sim,RoadSplits);
}

//-------------------------------------------------------------
void ResetNextRoadPtrs(Simulator * Sim)
{
	int i,j,k;
	// Connect the Roads
	for(i=0;i<Sim->numRoads;i++){		
		Sim->Roads[i].nextRoads.clear();
		for(j=0;j<(int)Sim->Roads[i].nextRoadIDs.size();j++)
			for(k=0;k<Sim->numRoads;k++)
				if(Sim->Roads[k].ID == Sim->Roads[i].nextRoadIDs[j]){
					Sim->Roads[i].nextRoads.push_back(&Sim->Roads[k]);
					continue;
				}
	}
}
//-------------------------------------------------------------
void SetRoadTilePtrs(Simulator * Sim, RNC * rnc)
{
	int k;
	// Set the Tile pointers in roads (has to be done after done with road splitting)
	for(k=0;k<(int) Sim->Roads.size();k++)
		Sim->Roads[k].SelfTilePtr = &rnc->Tiles[FindID<Tile>(rnc->Tiles,Sim->Roads[k].TileID)];
}

//-------------------------------------------------------------
void InitVectorMap(Simulator * Sim,VectorMap * RoadSplits)
{
	int k;

	for(k = 0;k<Sim->numRoads;k++)
		(*RoadSplits)[Sim->Roads[k].ID].push_back(Sim->Roads[k].ID);
}

//-------------------------------------------------------------
void ResetMSRoads(Simulator * Sim, VectorMap & Mapping)
{
	int k,l;
	int InitRdID;
	vector<int> * v;
	Road * rd;

	for(k=0;k<Sim->numMS;k++){

		// Find the mapping between the road to which the MS was originally connected and the split roads
		InitRdID = Sim->MS[k].InitRoadID;

		if(InitRdID<0 || !Sim->MS[k].isMobile) //Fixed UE, is not associated with a road
			continue;

		v = &(Mapping[InitRdID]); 

		if(v->size() <1)
			throw "Mobile Station original Road mapping not found !";

		// If there is only one member in the mapping, there is no need to compare distances, just find the Road with this ID:
		if(v->size() == 1){ 
			Sim->MS[k].CurrentRoad = &(Sim->Roads[FindID<Road>(Sim->Roads,(*v)[0])]);
		}

		//  Otherwise, compare the distance between MS and the roads (in the middle), and pick the closest one to associate to:
		else{ 
			Sim->MS[k].CurrentRoad  = NULL;
			for(l=0;l<(int)v->size();l++){
				rd = &(Sim->Roads[FindID<Road>(Sim->Roads,(*v)[l])]);

				if(Sim->MS[k].X>= min(rd->StrtEndPts[0],rd->StrtEndPts[2]) && Sim->MS[k].X<= max(rd->StrtEndPts[0],rd->StrtEndPts[2]) ){
					Sim->MS[k].CurrentRoad = rd;
					break;
				}

			}

			if(Sim->MS[k].CurrentRoad ==NULL)
				throw "Error connecting MS to split road";

		}
	}
}
