#pragma once
#include <Windows.h>
BOOL DirectoryExists(const char* szPath)
{
	DWORD dwAttrib = GetFileAttributes(szPath);

	return (dwAttrib != INVALID_FILE_ATTRIBUTES &&
		(dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
}

bool checkDirectory(char* directoryPath, char* extension,vector<char*> &replayPaths)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR *szDir=directoryPath;
	hFind = FindFirstFile(szDir, &ffd);
	
	DWORD creationTime=0;
	char ext[60];

   do
   {
      if ((ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) &&(!(ffd.dwFileAttributes&FILE_ATTRIBUTE_HIDDEN)) )
      {
		  if (strcmp(ffd.cFileName,".")&&strcmp(ffd.cFileName,".."))
		  {
				char tempDirectory[600];
				strcpy(tempDirectory,szDir);
				tempDirectory[strlen(tempDirectory)-2]=0;
				strcat(tempDirectory,"/");

				strcat(tempDirectory,ffd.cFileName);
				strcat(tempDirectory,"/*");
				if (checkDirectory(tempDirectory, extension, replayPaths))
					return true;
		  }
      }
		else
	  {
		int size = strlen( ffd.cFileName);

		if (strcmp(strlwr(ffd.cFileName+size-strlen(extension)),extension)==0)
		{
			char* tempS = new char [500];
			replayPaths.push_back(tempS);
			strcpy(tempS,directoryPath);
			tempS[strlen(tempS)-1]=0;
			strcat(tempS, ffd.cFileName);
		}
	  }	  
   }
   while (FindNextFile(hFind, &ffd) != 0);
   return false;
}
void getMapProblemFiles(bool getStaticMapProblems, char * mapsDirectory, vector<char*> &replayPaths)
{
	char * newMapDir = new char[strlen(mapsDirectory)+4];
	strcpy(newMapDir, mapsDirectory);
	if (DirectoryExists(mapsDirectory))
	{
		strcat(newMapDir, "/*");
		if (getStaticMapProblems)
		{
			checkDirectory(newMapDir, ".scen", replayPaths);
		}
		else
		{
			checkDirectory(newMapDir, ".dscen", replayPaths);		
		}
	}
	delete[] newMapDir;
}
void writeOutPathStats(char* scenarioName,int problemId,float pathDist,double time)
{
	static FILE* fp = fopen("pathStats.txt","w");
	fprintf(fp,"%s %d %f %f\n",scenarioName,problemId,pathDist,time);
	fflush(fp);
}
float getPathLen(std::vector<pair<short,short> >& thePath)
{
	float dist=0;
	if (thePath.size()==0)
		return 0;
	for (int i =0;i<thePath.size()-1;i++)
	{
		dist += sqrt(((float)(thePath[i].first - thePath[i + 1].first)*(thePath[i].first - thePath[i + 1].first) + (float)(thePath[i].second - thePath[i + 1].second)*(thePath[i].second - thePath[i + 1].second)));
	}
	return dist;
}