/* Author: Vladimir Korshak (c) 2014
* Indentaion: 2 Characters
* Standard: C++11
* Compiler: GCC 4.8
* Compilation: make
* Run UnitTest: ./PathfinderUnitTest [test_number]
*/

#include "Pathfinder.h"

#include <iostream>
#include <fstream>

/* Task description
Implement a path-finding algorithm in C++ that finds and outputs a shortest path
between start and target using the following function declaration:

int FindPath( const int nStartX, const int nStartY, const int nTargetX, const
int nTargetY, const unsigned char* pMap, const int nMapWidth, const int
nMapHeight, int* pOutBuffer, const int nOutBufferSize );

- nStartX and nStartY are the coordinates of the start position.
- nTargetX and nTargetY are the coordinates of the target position.
- pMap is a nMapWidth * nMapHeight large grid where traversable locations are
marked with 1 and non-traversable as 0. Locations are considered to be adjacent
horizontally and vertically but not diagonally.
- pOutBuffer is where you should output the found path and nOutBufferSize is the
maximum length of a path that can be written to pOutBuffer. Entries in
pOutBuffer are indices into pMap.
- The function should return the length of the path between Start and Target or
-1 if no path exists. If the path is larger than nOutBufferSize, the calling
function can either abort or choose to call FindPath again with a larger out
buffer.
- Consider performance, memory usage and that your code may be called from a
multi-threaded environment.
*/


/* Implementation specifics
* Algorhitm of choice: Waveform with custom enchancements
*
* The reason I've chosen against more popular A* is due to the nature(congestion)
* and size of the map are unknown. Certainly there are other algorithms that are
* more suitable for every given condition, be it congestion, size, computational
* intensity or memory footprint. Judging by the games that I've played recently,
* most of the maps don't have specific pitfalls that prevent waveform algorithm
* from finding path from A to B in reasonable amount of time and resources.
*
* In addition I've decided to go a little further and made some enchancements:
* - Multy-threaded approach to searching
* - Targeted waveform using Bounding box
* - Google Performance Tools were used for CPU and Memory profiling
*/



template<class T>
std::ostream& binary_write(std::ostream& stream, const T& value){
    return stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

template<class T>
std::istream& binary_read(std::istream& stream, T& value){
    return stream.read(reinterpret_cast<char*>(&value), sizeof(T));
}

void SaveMapToFile(std::string path, const unsigned char* pMap, const unsigned int nMapSize)
{
  // Save a map that is sucessfully generated and is traversable
  std::ofstream stream(path, std::ios::binary);
  printf("Saving map to file...");
  for (unsigned int i = 0; i< nMapSize; i++)
  {
    binary_write(stream, pMap[i]);
  }  
  stream.close();
  printf("Done\n");
}

void LoadMapFromFile(std::string path, unsigned char* pMap, const unsigned int nMapSize)
{
  // Load a previously saved map
  std::ifstream inStream(path, std::ios::binary);
  printf("Loading Map from file...");
  for (unsigned int i = 0; i< nMapSize; i++)
  {
    binary_read(inStream, pMap[i]);
  }
  
  inStream.close();
  printf("Done\n");
}


void GenerateRandomMap(const unsigned int nStartX, const unsigned int nStartY,
                       const unsigned int nTargetX, const unsigned int nTargetY,
                       unsigned char* Map,
                       const unsigned int nMapWidth, const unsigned int nMapHeight)
{
  char rnd = 0;

  // initialize random seed
  srand (time(NULL));
  for (unsigned int i=0; i<nMapWidth*nMapHeight; ++i)
  {
    rnd = rand() % 5;
    if (rnd <= 2) Map[i] = 1;
    else Map[i] = 0;
  }

  // set start and target to traversable
  Map[nStartY*nMapHeight+nStartX] = 1;
  Map[nTargetY*nMapHeight+nTargetY] = 1;
}


void UnitTest_LoadSaveMap()
{
  printf("\n\n~~~ Load Save Unit test ~~~ \n");

  unsigned int nMapSizeBytes = 10*10;
  unsigned int nMapWidth = std::floor(std::sqrt(nMapSizeBytes));
  unsigned int nMapHeight = nMapWidth;

  unsigned char Map[]={ 1,0,1,1,1,0,1,1,1,1,
                        1,0,1,0,1,0,1,0,0,1,
                        1,0,1,0,1,0,1,0,1,1,
                        1,0,1,0,1,0,1,0,1,0,
                        1,0,1,0,1,0,1,0,1,1,
                        1,0,1,0,1,0,1,0,0,1,
                        1,0,1,0,1,0,1,0,1,1,
                        1,0,1,0,1,0,1,0,1,0,
                        1,0,1,0,1,0,1,0,1,1,
                        1,1,1,0,1,1,1,0,0,1 };

  std::string path("LoadSaveTest.map");
  printf("~~~ Before load ~~~\n");
  for (unsigned int y=0; y<nMapHeight; ++y)
  {
    printf("FindPath | PrintMap | ");
    for (unsigned int x=0; x<nMapWidth; ++x)
    {
      if ( Map[y*nMapHeight+x] == 1) printf("_ ");
      else printf("# ");
    }
    printf(" |\n");
  }

  SaveMapToFile(path, Map, nMapSizeBytes);
  
  printf("\n~~~ Nullifying the array ~~~\n");
    
  for (unsigned int i=0; i<nMapSizeBytes; ++i)
  {
    Map[i] = 0;
  }

  for (unsigned int y=0; y<nMapHeight; ++y)
  {
    printf("FindPath | PrintMap | ");
    for (unsigned int x=0; x<nMapWidth; ++x)
    {
      if ( Map[y*nMapHeight+x] == 1) printf("_ ");
      else printf("# ");
    }
    printf(" |\n");
  }  
  

  LoadMapFromFile(path, Map, nMapSizeBytes);

  printf("\n~~~ After Load ~~~\n");
  for (unsigned int y=0; y<nMapHeight; ++y)
  {
    printf("FindPath | PrintMap | ");
    for (unsigned int x=0; x<nMapWidth; ++x)
    {
      if ( Map[y*nMapHeight+x] == 1) printf("_ ");
      else printf("# ");
    }
    printf(" |\n");
  }
  
}  

void UnitTest_Pathfinder(std::string path, unsigned int nMapSizeBytes,
                         unsigned int nOutBufferSize,
                         bool bUseMultipleThreads, bool bUseManhattanBbox)
{
    unsigned int nMapWidth = std::floor(std::sqrt(nMapSizeBytes));
    unsigned int nMapHeight = nMapWidth;

    unsigned int nStartX = 0;
    unsigned int nStartY = 0;
    unsigned int nTargetX = nMapWidth-1;
    unsigned int nTargetY = nMapHeight-1;
    
    printf("\n~~~ UnitTest_Pathfinder ~~~ \n");
    printf("Map filename: %s\n", path.c_str());
    printf("Map size(bytes): %d\n", nMapSizeBytes);
    printf("Map dimemsions: %dx%d\n", nMapWidth, nMapHeight);
    printf("Start (%d,%d)    Target (%d,%d)\n", nStartX, nStartY, nTargetX, nTargetY);
    printf("Use Manhattan Bbox Optimization: %s\n", bUseManhattanBbox ? "true" : "false");
    printf("Use Multiple Threads Optimization: %s\n", bUseMultipleThreads ? "true" : "false");
    
    unsigned char* pMap;
    int* OutBuffer;

    try
    {
      printf("Allocating memory...");
      pMap = new unsigned char[nMapWidth*nMapHeight];
      OutBuffer = new int[nOutBufferSize];
      printf("Done\n");      
    }
    catch (std::bad_alloc& ba)
    {
      printf("The program is not able to allocate that much memory: %s\n", ba.what());
      abort();
    }

    
    LoadMapFromFile(path, pMap, nMapSizeBytes);

    cPathfinder* pf = new cPathfinder(bUseMultipleThreads, bUseManhattanBbox);

    printf("Starting the search...From Corners.\n");
    
    // set the clock
    auto start = std::chrono::system_clock::now();

    pf->FindPath(nStartX, nStartY, nTargetX, nTargetY,
                 pMap, nMapWidth, nMapHeight,
                 OutBuffer, nOutBufferSize);

    auto end = std::chrono::system_clock::now();
    
    
    printf("Execution time: %lld milliseconds\n",
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    delete pf;
    
    nStartX = 3854;
    nStartY = 3854;
    nTargetX = 6432;
    nTargetY = 6425;
    
    pf = new cPathfinder(bUseMultipleThreads, bUseManhattanBbox);
    
    printf("Starting the search...From points in the middle.\n");

    // set the clock
    start = std::chrono::system_clock::now();

    pf->FindPath(nStartX, nStartY, nTargetX, nTargetY,
                 pMap, nMapWidth, nMapHeight,
                 OutBuffer, nOutBufferSize);

    end = std::chrono::system_clock::now();

    printf("Execution time: %lld milliseconds\n",
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());


    delete pf;
    delete [] OutBuffer;
    delete [] pMap;
}




int main(int argc, const char* argv[])
{
  std::cout << "\n~~~ cPathfinder Unit Test ~~~\n" << std::endl;
  
   // ok

  std::string path = "100mbTraversable.map";
  unsigned int nMapSizeBytes  = 100000000;  // 100mb
  unsigned int nOutBufferSize = 30000;   // Lets say its unlimited
  int choice = 9;
  if (argc == 2) choice = atoi(argv[1]);
  printf("choice: %d", choice);
  
  switch (choice){
    case 0: UnitTest_LoadSaveMap(); break;
    case 1: UnitTest_Pathfinder(path, nMapSizeBytes, nOutBufferSize, false, false); break;
    case 2: UnitTest_Pathfinder(path, nMapSizeBytes, nOutBufferSize, false, true); break;
    case 3: UnitTest_Pathfinder(path, nMapSizeBytes, nOutBufferSize, true, true); break;
    case 4: UnitTest_Pathfinder(path, nMapSizeBytes, nOutBufferSize, true, false); break;
    default: printf("No option specified\n");
  }

  return 0;
}