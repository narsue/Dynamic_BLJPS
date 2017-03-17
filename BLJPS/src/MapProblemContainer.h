#pragma once
#include "getFileNames.h"
#include "MapGridData.h"
#include "Rect.h"
#include <vector>
#include <string>

struct MapPathQuery
{
	MapPathQuery(int _sx, int _sy, int _dx, int _dy)
	{
		sx = _sx;
		sy = _sy;
		dx = _dx;
		dy = _dy;
	}
	int sx, sy, dx, dy;
};

class MapProblemContainer
{
	int currentProblemId;
	std::vector<char*> problemFileNames;
	vector<MapPathQuery> queryLocations;
	vector<Rect> dynamicObstacles;
	MapGridData mapData;
	string currentMapName;
	string mapsDirectory;
	bool staticMapData;
	bool loadStaticMapData(char * sceneFileName)
	{
		queryLocations.clear();
		FILE* fpProblemSet = fopen(sceneFileName, "r");
		if (fpProblemSet == 0)
		{
			printf("Failed to load problem file %s\n", sceneFileName);
			system("pause");
			return false;
		}
		fscanf(fpProblemSet, "%*s %*f"); //Version 1.0	
		char fileName[512];
		int sX, sY, eX, eY;
		while (!feof(fpProblemSet))
			if (fscanf(fpProblemSet, "%*d %s %*d %*d %d %d %d %d %*f", fileName, &sX, &sY, &eX, &eY) != 5)
				break;
			else
				queryLocations.push_back(MapPathQuery(sX, sY, eX, eY));
		fclose(fpProblemSet);

		currentMapName = string(mapsDirectory);
		currentMapName += "/";
		currentMapName += fileName+5;
		if (!mapData.fillGrid(currentMapName.c_str()))
		{
			printf("Failed to load map file %s associated with the problem file %s\n", fileName, sceneFileName);
			system("pause");
			return false;
		}
		return true;
	}
	bool loadDynamicMapData(char * dynamicSceneFileName)
	{
		dynamicObstacles.clear();
		FILE* fpProblemSet = fopen(dynamicSceneFileName, "r");
		if (fpProblemSet == 0)
		{
			printf("Failed to load problem file %s\n", dynamicSceneFileName);
			system("pause");
			return false;
		}
		fscanf(fpProblemSet, "%*s %*f"); //Version 1.0	
		char fileName[512];
		int x, y, w, h;
		while (!feof(fpProblemSet))
			if (fscanf(fpProblemSet, "%*d %s %d %d %d %d", fileName, &x, &y, &w, &h) != 5)
				break;
			else
				dynamicObstacles.push_back(Rect(x, y + h, x + w, y));

			//if (fscanf(fpProblemSet, "%*d %s %*d %*d %d %d %d %d %*f", fileName, &sX, &sY, &eX, &eY) != 5)
			//	break;
			//else
			//	queryLocations.push_back(MapPathQuery(sX, sY, eX, eY));
		fclose(fpProblemSet);
		string sceneFileName = mapsDirectory + (fileName + 7);

		

		return loadStaticMapData((char*)sceneFileName.c_str());
	}

public:
	MapProblemContainer(bool staticMaps, char * _mapsDirectory) :mapsDirectory(_mapsDirectory)
	{
		mapsDirectory = _mapsDirectory;
		staticMapData = staticMaps;
		getMapProblemFiles(staticMaps, (char*)mapsDirectory.c_str(), problemFileNames);
		if (problemFileNames.size() && 
			((staticMaps && loadStaticMapData(problemFileNames[0])) || (!staticMaps && loadDynamicMapData(problemFileNames[0]))))
		{
			currentProblemId = 0;
		}
		else
		{
			currentProblemId = -1;
			printf("Failed to load any problems directory may be incorrect\n");
			printf("Directory: %s\n", _mapsDirectory);
			system("pause");
		}
	}
	~MapProblemContainer()
	{
		for (int i = 0; i < problemFileNames.size(); i++)
			delete[] problemFileNames[i];
	}
	void getStaticMapData(int mapId, char * &rawMapData, int &width, int &height)
	{
		if (currentProblemId != mapId)
		{
			loadStaticMapData(problemFileNames[mapId]);
			currentProblemId = mapId;
		}
		rawMapData = mapData.gridDataCopy;
		width = mapData.gridWidth;
		height = mapData.gridHeight;
	}
	void getDynamicMapData(int mapId, char * &rawMapData, int &width, int &height, vector<Rect> & _dynamicObstacles)
	{
		if (currentProblemId != mapId)
		{
			loadDynamicMapData(problemFileNames[mapId]);
			currentProblemId = mapId;
		}
		rawMapData = mapData.gridDataCopy;
		width = mapData.gridWidth;
		height = mapData.gridHeight;
		_dynamicObstacles = dynamicObstacles;
	}

	MapPathQuery getQueryId(int id)
	{
		return queryLocations[id];
	}
	int getNumberOfMaps()
	{
		return problemFileNames.size();
	}
	int getNumberOfQueriesForMap(int mapId)
	{
		if (mapId<0||mapId > getNumberOfMaps())
			return 0;
		if (currentProblemId != mapId)
		{
			loadStaticMapData(problemFileNames[mapId]);
			currentProblemId = mapId;
		}

		return queryLocations.size();
	}
	int getNumQueries()
	{
		return queryLocations.size();
	}
	const char * getCurrentMapName()
	{
		return currentMapName.c_str();
	}
};