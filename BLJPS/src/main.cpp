#include "Astar.h"
#include "BL_JPS.h"
#include "JPS.h"
#include "JPS_Plus.h"
#include "BL_JPS_PLUS.h"
#include "MapGridData.h"
#include "getFileNames.h"
#include "MapProblemContainer.h"
#include "Timer.h"

#define WRITE_SPEED_UP_TO_FILE
#define MAP_DIRECTORY "../../maps"
double benchmarkQuery(PathFindingAlgorithm * algorithm, MapPathQuery &query, vector<Coordinate> &solution)
{
	Timer t;
	t.StartTimer();
	int iterations = 0;
	do
	{
		for (int repeatQueryId = 0; repeatQueryId < 100; ++repeatQueryId)
			algorithm->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
		iterations += 100;
	} while (t.GetTime() < 5);
	return (double)t.EndTimer() / iterations;
}
FILE* openSpeedResultsFile(PathFindingAlgorithm * algorithm,bool staticMaps,int numPaths=1)
{
	char fileName[128];
	if (staticMaps)
		sprintf(fileName, "../results/SpeedResults_Static_%s.txt", algorithm->getAlgorithmName());
	else
		sprintf(fileName, "../results/SpeedResults_Dynamic_%d_%s.txt", numPaths,algorithm->getAlgorithmName());

	FILE* fp = fopen(fileName, "w");
	if (fp == 0)
	{
		printf("Failed to open the file %s. Create the folder or check folder permissions\n", fileName);
		system("pause");
	}
	return fp;
}
void writeBenchMarkToFile(PathFindingAlgorithm * algorithm, MapPathQuery &query, FILE* fileOutHandle,const char * mapName, int problemId)
{
	vector<Coordinate> solution;
	double timeTaken = benchmarkQuery(algorithm, query, solution);
	if (fileOutHandle)
		fprintf(fileOutHandle, "%s %d %f %f\n", mapName, problemId, pathLength(solution), timeTaken);
	//if (problemId%200==0)
	//	printf("timeTaken:%f\n", timeTaken);
}
PathFindingAlgorithm * getAlgorithmType(AlgorithmType algType, char * mapData, int mapWidth, int mapHeight)
{
	if (algType == AT_JPS)
		return (new JPS(mapData, mapWidth, mapHeight));
	if (algType == AT_BL_JPS)
		return (new BL_JPS(mapData, mapWidth, mapHeight));
	if (algType == AT_JPS_PLUS)
		return (new JPS_PLUS(mapData, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_PLUS)
		return (new BL_JPS_PLUS(mapData, mapWidth, mapHeight));
	if (algType == AT_ASTAR)
		return (new AStar(mapData, mapWidth, mapHeight));
	
	return 0;
}
void addAlgorithmToTest(AlgorithmType algType, vector<FILE *> &speedUpResults, vector<AlgorithmType> &algorithms)
{
	algorithms.push_back(algType);
	speedUpResults.push_back(0);
}
void testAlgorithmSolutions(AlgorithmType algTypeA, AlgorithmType algTypeB)
{
	MapProblemContainer mapProblems(true, MAP_DIRECTORY);

	for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)
	{
		char * mapData = 0;
		int mapWidth = 0, mapHeight = 0;
		mapProblems.getStaticMapData(mapId, mapData, mapWidth, mapHeight);

		//Create the BL-JPS algorithm with the map data, width and height
		PathFindingAlgorithm * algorithmA = getAlgorithmType(algTypeA, mapData, mapWidth, mapHeight);
		PathFindingAlgorithm * algorithmB = getAlgorithmType(algTypeB, mapData, mapWidth, mapHeight);
		printf("%s: Map %d: (%d,%d) %s\n", algorithmA->getAlgorithmName(), mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

		//Preprocess the map
		algorithmA->preProcessGrid();
		algorithmB->preProcessGrid();
		for (int queryId = 0; queryId < mapProblems.getNumQueries(); queryId++)
		{
			MapPathQuery query = mapProblems.getQueryId(queryId);

			vector<Coordinate> solution,solution2;
			algorithmA->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
			algorithmB->findSolution(query.sx, query.sy, query.dx, query.dy, solution2);

			if (nonSimilarFloat(pathLength(solution), pathLength(solution2)))
			{
				printf("Path lengths don't match %d %d %d %d\n", mapId, queryId, solution.size(), solution2.size());
			}
		}

		//Delete the algorithm after it has finished a map, recreate it for new maps
		delete algorithmA;
		delete algorithmB;
	}
	
}
void runAllDynamicMapsAndQueries(int pathQueriesPerObstacleChange, AlgorithmType algType)
{
#ifdef _DEBUG
	printf("Benchmark algorithms in release mode!\n");
	return;
#endif
	MapProblemContainer mapProblems(false, MAP_DIRECTORY);

	vector<FILE *> speedUpResults;
	vector<AlgorithmType> algorithms;
	addAlgorithmToTest(algType, speedUpResults, algorithms);

	for (int algorithmId = 0; algorithmId < algorithms.size(); algorithmId++)
	{
		for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)
		{
			char * mapData = 0;
			int mapWidth = 0, mapHeight = 0;
			//Describes a set of rectangular obstacles that will  be added and removed on the current map
			vector<Rect> dynamicObstacles;

			//Get the map data for the mapId
			mapProblems.getDynamicMapData(mapId, mapData, mapWidth, mapHeight, dynamicObstacles);

			//Benchmark the first 30 obstacles of size 1-3 in linear dimension
			const int objectCount = dynamicObstacles.size()<30 ? dynamicObstacles.size():30;

			//Create the BL-JPS algorithm with the map data, width and height
			PathFindingAlgorithm * algorithm = getAlgorithmType(algorithms[algorithmId], mapData, mapWidth, mapHeight);
			printf("%s: Map %d: (%d,%d) %s\n", algorithm->getAlgorithmName(), mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

#ifdef WRITE_SPEED_UP_TO_FILE
			if (speedUpResults[algorithmId] == 0)
				speedUpResults[algorithmId] = openSpeedResultsFile(algorithm, false, pathQueriesPerObstacleChange);
#endif

			//Preprocess the map
			algorithm->preProcessGrid();
			//backup the preprocessed data to save us preprocessing the map every set of pathQueriesPerObstacleChange.
			algorithm->backupPreProcess();
			
			
			int startPath = 0;
			//Iterate over all the paths in the map file making sure we can do the batch size Ie skip the remainder if its less than pathQueriesPerObstacleChange
			while (startPath + pathQueriesPerObstacleChange < mapProblems.getNumQueries())
			{
				//Start the timer
				Timer t;
				t.StartTimer();
				int iterations = 0;
				vector<Coordinate> solution;
				//repeat the path queries until 5ms has passed (for accurate measurement of fast path queries)
				do
				{
					//repeat the path queries at least 100 times has passed (for accurate measurement of path queries)
					for (int queryLoopId = 0; queryLoopId < 100; queryLoopId++)
					{
						//Alternate between adding and removign the obstacles 
						for (int objectId = 0; objectId < objectCount && objectId < dynamicObstacles.size(); objectId++)
						{
							if (queryLoopId % 2 == 0)
							{
								bool p = false;
								for (int y = dynamicObstacles[objectId]._trY; y < dynamicObstacles[objectId]._blY; y++)
									for (int x = dynamicObstacles[objectId]._blX; x < dynamicObstacles[objectId]._trX; x++)
									{
										int index = y*mapWidth + x;
										mapData[index / 8] |= (1 << index % 8);
									}
								algorithm->reProcessGrid(dynamicObstacles[objectId]._blX, dynamicObstacles[objectId]._trX, dynamicObstacles[objectId]._trY, dynamicObstacles[objectId]._blY);
							}
							else
							{
								for (int y = dynamicObstacles[objectId]._trY; y < dynamicObstacles[objectId]._blY; y++)
									for (int x = dynamicObstacles[objectId]._blX; x < dynamicObstacles[objectId]._trX; x++)
									{
										int index = y*mapWidth + x;
										mapData[index / 8] = mapData[index / 8] & ~(1 << index % 8);
									}
								algorithm->reProcessGrid(dynamicObstacles[objectId]._blX, dynamicObstacles[objectId]._trX, dynamicObstacles[objectId]._trY, dynamicObstacles[objectId]._blY);
							}
						}

						//Notify the algorithm that all changes to the map have been completed and we are about to query paths
						algorithm->flushReProcess();

						//Batch query paths 
						for (int queryId = startPath; queryId < mapProblems.getNumQueries() && queryId < startPath + pathQueriesPerObstacleChange; queryId++)
							algorithm->findSolution(mapProblems.getQueryId(queryId).sx, mapProblems.getQueryId(queryId).sy, mapProblems.getQueryId(queryId).dx, mapProblems.getQueryId(queryId).dy, solution);
					}
					iterations += 100;
				}	
				while (t.GetTime() < 5);

				//Record the time taken to query the set of paths
				double timeTaken = (double)t.EndTimer() / iterations;
				if (speedUpResults[algorithmId])
					fprintf(speedUpResults[algorithmId], "%s %d %f %f\n", mapProblems.getCurrentMapName(), startPath, pathLength(solution), timeTaken);
				
				//Use the backup data that was recorded after initialy preprocessing the map. Speeds up result gathering as we don't have to reprocess the map again
				algorithm->useBackupData();

				//Increment the starting offset of the path quiery ids
				startPath += pathQueriesPerObstacleChange;
			}

			//Flush the result file for each map
			if (speedUpResults[algorithmId])
				fflush(speedUpResults[algorithmId]);

			//Delete the algorithm after it has finished a map, recreate it for new maps
			delete algorithm;
		}
		if (speedUpResults[algorithmId])
		{
			fclose(speedUpResults[algorithmId]);
			speedUpResults[algorithmId] = 0;
		}
	}
}
void runAllStaticMapsAndQueries()
{
#ifdef _DEBUG
	printf("Benchmark algorithms in release mode!\n");
	return ;
#endif
	MapProblemContainer mapProblems(true, MAP_DIRECTORY);
	
	vector<FILE *> speedUpResults;
	vector<AlgorithmType> algorithms;
	addAlgorithmToTest(AT_BL_JPS, speedUpResults, algorithms);
	addAlgorithmToTest(AT_JPS, speedUpResults, algorithms);

	for (int algorithmId = 0; algorithmId < algorithms.size(); algorithmId++)
	{
		for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)//mapProblems.getNumberOfMaps()
		{
			char * mapData = 0;
			int mapWidth = 0, mapHeight = 0;
			mapProblems.getStaticMapData(mapId, mapData, mapWidth, mapHeight);

			//Create the BL-JPS algorithm with the map data, width and height
			PathFindingAlgorithm * algorithm = getAlgorithmType(algorithms[algorithmId], mapData, mapWidth, mapHeight);
			printf("%s: Map %d: (%d,%d) %s\n", algorithm->getAlgorithmName(),mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

			#ifdef WRITE_SPEED_UP_TO_FILE
			if (speedUpResults[algorithmId] == 0)
				speedUpResults[algorithmId] = openSpeedResultsFile(algorithm,true);
			#endif

			//Preprocess the map
			algorithm->preProcessGrid();

			for (int queryId = 0; queryId < mapProblems.getNumQueries(); queryId++)
				writeBenchMarkToFile(algorithm, mapProblems.getQueryId(queryId), speedUpResults[algorithmId], mapProblems.getCurrentMapName(), queryId);

			//Delete the algorithm after it has finished a map, recreate it for new maps
			delete algorithm;
		}
		if (speedUpResults[algorithmId])
		{
			fclose(speedUpResults[algorithmId]);
			speedUpResults[algorithmId] = 0;
		}
	}
}
void testSingleMapPath(AlgorithmType algType, int mapId, int queryId)
{
	//Specifices the location where all the map files are stored
	//Caches all the available mpa file names
	MapProblemContainer mapProblems(true, MAP_DIRECTORY);

	//Next we request the data for the given mapId
	//The data we get back is the width and height of the map and a bit packed copy of the map contents
	char * mapData = 0;
	int mapWidth = 0, mapHeight = 0;
	mapProblems.getStaticMapData(mapId, mapData, mapWidth, mapHeight);

	//Creates the algorithm requested - see the definition of AlgorithmType for all algorithms
	PathFindingAlgorithm * algorithm = getAlgorithmType(algType, mapData, mapWidth, mapHeight);

	//Preprocesses the grid if required
	algorithm->preProcessGrid();

	//Gets the query for id requested, its basically just a start and destination on the given map
	MapPathQuery query = mapProblems.getQueryId(queryId);
	printf("Run query on map %s (%d,%d)->(%d,%d)\n", mapProblems.getCurrentMapName(), query.sx, query.sy, query.dx, query.dy);
	vector<Coordinate> solution;
	
	//Runs the algorithm for the given query, it returns a vector of Coordinates that make up the path
	algorithm->findSolution(query.sx, query.sy, query.dx, query.dy, solution);

	printf("Found a solution of length %f\n", pathLength(solution));
}



int main()
{
	//If its your first look at this project start by examining testSingleMapPath
	//Test a single path, this is a simple example that will help you get up and running quickly
	//testSingleMapPath(AT_JPS, 0, 11);//AT_JPS


	//This function will iterate over all the queries for all the maps and write the results to file if WRITE_SPEED_UP_TO_FILE is defined
	//runAllStaticMapsAndQueries();
	
	//Tests the results path lengths between two algorithms over all the maps and their paths
	//testAlgorithmSolutions(AT_BL_JPS, AT_JPS_PLUS);

	//Run the dynamic map problems
	//Run 100 paths before updating the obstacles
	//runAllDynamicMapsAndQueries(100, AT_ASTAR);
	//runAllDynamicMapsAndQueries(100, AT_JPS);
	runAllDynamicMapsAndQueries(100, AT_BL_JPS);

	//To change from FLUSH update to Partial Update undefine the flags BLJPSPLUS_FLUSH_UPDATE/JPSPLUS_FLUSH_UPDATE
	//These are located at the top of each algorithms header file 
	//runAllDynamicMapsAndQueries(100, AT_JPS_PLUS);
	//runAllDynamicMapsAndQueries(100, AT_BL_JPS_PLUS);

	system("pause");

	return 0;
}