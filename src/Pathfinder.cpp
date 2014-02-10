#include "Pathfinder.h"


cPathfinderWorker::cPathfinderWorker()
{
}

cPathfinderWorker::~cPathfinderWorker()
{
}

cPathfinder::cPathfinder(bool _bUseMultipleThreads=false,
                         bool _bUseManhattanBbox=false)
{
  bUseMultipleThreads = _bUseMultipleThreads;
  bUseManhattanBbox   = _bUseManhattanBbox;
}


int cPathfinder::FindPath(const int nStartX, const int nStartY,
                          const int nTargetX, const int nTargetY,
                          const unsigned char* pMap,
                          const int nMapWidth, const int nMapHeight,
                          int* pOutBuffer, const int nOutBufferSize)
{
  nResult = 0;
  
  if (bUseMultipleThreads)
  {
    // Multithreaded implementation

    // indicator variables that controls the search
    bool bWorkerStartEndIsSearching = true;
    bool bWorkerEndStartIsSearching = true;

    Node  nHandshake = Node(-1,-1);

    bool bHandshakeFound = false;
    
    // Data structures that get filled by both searches
    std::vector<Node> vecStartEnd;
    std::vector<Node> vecEndStart;
    
    int* workerSEbuffer;
    int* workerESbuffer;

    printf("Starting Pathfinder workers...");
    cPathfinderWorker* workerStartEnd = new cPathfinderWorker();
    workerStartEnd->UseMultiThread(true, &vecStartEnd, &bWorkerStartEndIsSearching);
    workerStartEnd->UseManhattanBBox(true, 3);

    cPathfinderWorker* workerEndStart = new cPathfinderWorker();
    workerEndStart->UseMultiThread(true, &vecEndStart, &bWorkerEndStartIsSearching);
    workerEndStart->UseManhattanBBox(true, 3);

    std::thread thStartEnd(&cPathfinderWorker::FindPath, workerStartEnd,
                                                   nStartX, nStartY, nTargetX, nTargetY,
                                                   pMap, nMapWidth, nMapHeight,
                                                   workerSEbuffer, nOutBufferSize);

    std::thread thEndStart(&cPathfinderWorker::FindPath, workerEndStart,
                                                   nStartX, nStartY, nTargetX, nTargetY,
                                                   pMap, nMapWidth, nMapHeight,
                                                   workerESbuffer, nOutBufferSize);

    printf("Done\n");
    printf("Waiting for handshake...\n");
    // perform constant search for the handshake between the two search waves
    while(!bHandshakeFound && bWorkerEndStartIsSearching && bWorkerStartEndIsSearching)
    {
      // only start the search once both searches reach midpoint
      if (!vecStartEnd.empty() && !vecEndStart.empty())
      {
//         printf("Midpoint vectors not empty");
        // we only perform reads and comparisons
        for (auto& it : vecStartEnd)
        {
          if (std::find(vecEndStart.begin(), vecEndStart.end(), it)!=vecEndStart.end())
          {
            // we have found an item that is present in both searches
            bHandshakeFound = true;
            bWorkerStartEndIsSearching = false;
            bWorkerEndStartIsSearching = false;
            nHandshake = it;
            break;
          }
        }
      }
      // make sure we are nice to others
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    printf("Search complete. Joining threads.\n");

    // Make sure we are in sync
    thStartEnd.join();
    thEndStart.join();

    
    printf("Threads joined.\n");
    if (bHandshakeFound)
    {

      try
      {
        printf("Allocating Memory for intermediate buffers...");
        workerSEbuffer = new int[nOutBufferSize];
        workerESbuffer = new int[nOutBufferSize];
        printf("Done\n");
      }
      catch (std::bad_alloc& ba)
      {
        printf("The program is not able to allocate that much memory: %s\n", ba.what());
        abort();
      }
      
      printf("Handshake is found. Reconstructing path...\n");
      unsigned int nHandshakeX = nHandshake.first;
      unsigned int nHandshakeY = nHandshake.second;


      // this could be multithreaded as well
      //thStartEnd(workerStartEnd.Reconstruct(nHandshakeX,nHandshakeY, workerSEbuffer));
      unsigned int wSEpath = workerStartEnd->Reconstruct(nHandshakeX,nHandshakeY, workerSEbuffer);
      unsigned int wESpath = workerEndStart->Reconstruct(nHandshakeX,nHandshakeY, workerESbuffer);

      
      // join the two sequences
      for (int i=0; i<static_cast<int>(wSEpath); ++i)
      {
        pOutBuffer[i] = workerSEbuffer[i];
      }
      for (int i=0; i<static_cast<int>(wESpath); ++i)
      {
        pOutBuffer[i+wSEpath] = workerESbuffer[i];
      }

      nResult = wSEpath + wESpath;
      printf("Reconstruction complete. Path length: %d\n", nResult);
    }
    else
    {
      printf("Handshake not found. Path does not exist\n");
    }
    
  }
  else
  {
    // Singlethreaded implementation
    cPathfinderWorker* worker = new cPathfinderWorker();

    if (bUseManhattanBbox)
    {
      worker->UseManhattanBBox(true, 3);
    }
    
    nResult = worker->FindPath(nStartX, nStartY, nTargetX, nTargetY,
                              pMap, nMapWidth, nMapHeight,
                              pOutBuffer, nOutBufferSize);
  }
  
  return nResult;
}




int cPathfinderWorker::FindPath(const int nStartX, const int nStartY,
                                const int nTargetX, const int nTargetY,
                                const unsigned char* pMap,
                                const int nMapWidth, const int nMapHeight,
                                int* pOutBuffer, const int nOutBufferSize)
{

    if (!bUseMultipleThreads)
    {
      bKeepSearching = new bool;
    }

    // if search succeeds, we reconstruct the path and return its length to the caller
    if ( Search(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, nOutBufferSize) )
    {
      // we reconstruct automatically only in single thread variant
      if (!bUseMultipleThreads) return Reconstruct(nTargetX, nTargetY, pOutBuffer);
    }
    return -1;
}




void cPathfinderWorker::UseMultiThread(bool _bMultithread, std::vector<Node>* _vecPastManhattan,
                                       bool* _bKeepSearching)
{
  bUseMultipleThreads  = _bMultithread;
  vecPastManhattan = _vecPastManhattan;
  bKeepSearching   = _bKeepSearching;
}

void cPathfinderWorker::UseManhattanBBox(bool _bUseManhattanBbox, unsigned int _nMDWindowSize)
{
  bUseManhattanBbox = _bUseManhattanBbox;
  nMDWindowSize     = _nMDWindowSize;
}

bool cPathfinderWorker::Search(const int nStartX, const int nStartY,
                        const int nTargetX, const int nTargetY,
                        const unsigned char* pMap,
                        const int _nMapWidth, const int _nMapHeight,
                        const int nOutBufferSize)
{
  bPathFound = false;
  *bKeepSearching = true;

  nMapHeight = _nMapHeight;
  nMapWidth  = _nMapWidth;
  
  std::deque<Node> NodesToVisit;
      
  unsigned int nStepCounter = 0;
  
  // Manhattan Bbox variables
  unsigned int nMDStartEnd = std::abs(nTargetX-nStartX) + std::abs(nTargetY-nStartY);

  DistanceMap.insert(std::make_pair(Node(nStartX,nStartY), nStepCounter));
  NodesToVisit.push_back(Node(nStartX,nStartY));

  // iterate until we are out of nodes within reach or path is found or we are told to stop
  while (!NodesToVisit.empty() && !bPathFound && *bKeepSearching)
  {
    Node NodesIter = NodesToVisit.front();
    NodesToVisit.pop_front();
    nStepCounter = DistanceMap.find(NodesIter)->second +1;
    
    // don't proceed in directions that surpass our path size limit
    if (static_cast<int>(nStepCounter) > nOutBufferSize){
      continue;
    }

    nCurrX = NodesIter.first;
    nCurrY = NodesIter.second;

    // iterate over the adjacent nodes
    for (int i=0; i<4; ++i){
      nAdjX = nCurrX+DIR[i].first;
      nAdjY = nCurrY+DIR[i].second;
      // Make sure the node is within our map boundaries
      if ( static_cast<int>(nAdjX) >= 0 && static_cast<int>(nAdjX) <= nMapWidth &&
          static_cast<int>(nAdjY) >= 0 && static_cast<int>(nAdjY) <= nMapHeight)
      {
        // Make sure the node is traversable
        if (pMap[nAdjY*nMapHeight+nAdjX] == 1)
        {
          
          // Further limits the range of out search using Manhattan distance to prevent
          // the wave from spreading in all directions infinitely
          if (bUseManhattanBbox)
          {
            nMDAdjEnd  = std::abs(nTargetX-nAdjX)  + std::abs(nTargetY-nAdjY);
            nMDCurrEnd = std::abs(nTargetX-nCurrX) + std::abs(nTargetY-nCurrY);

            if (nMDAdjEnd > nMDCurrEnd && nMDAdjEnd > nMDStartEnd + nMDWindowSize) continue;
          }

          // In case if we are working in pair with some other search,
          // save the nodes that appear past Manhattan Distance midpoint into given location
          if (bUseMultipleThreads)
          {
            nMDAdjEnd = std::abs(nTargetX-nAdjX)  + std::abs(nTargetY-nAdjY);

            if (nMDAdjEnd <= nMDStartEnd / 2) vecPastManhattan->push_back(Node(nCurrX, nCurrY));
          }
          
          // check if we have found our destination
          if (static_cast<int>(nAdjX) == nTargetX && static_cast<int>(nAdjY) == nTargetY) bPathFound = true;
                                  
          // Make sure we store the shortest possible path to this node
          auto prevValue = DistanceMap.find(Node(nAdjX,nAdjY));
          if (prevValue != DistanceMap.end())
          {
            if (prevValue->second > nStepCounter) {
              // modify the value and mark this node for revisiting
              DistanceMap[prevValue->first] = nStepCounter;
              if (static_cast<int>(nStepCounter)+1<=nOutBufferSize) NodesToVisit.push_back(Node(nAdjX, nAdjY));
            }
          }
          else
          {
            DistanceMap.insert(std::make_pair(Node(nAdjX,nAdjY), nStepCounter));
            if (static_cast<int>(nStepCounter)+1<=nOutBufferSize) NodesToVisit.push_back(Node(nAdjX, nAdjY));
          }
        }
      }
    }
  } // done iterating over the NodesToVisit
  printf("Result: %s, Nodes Checked %d\n", bPathFound ? "true" : "false", DistanceMap.size());
  *bKeepSearching = false;
  return bPathFound;
}


int cPathfinderWorker::Reconstruct(const int nTargetX, const int nTargetY, int* pOutBuffer)
{
  int nResult = 0;
  
  auto itDL = DistanceMap.find(Node(nTargetX, nTargetY));
  if (itDL != DistanceMap.end())
  {
    // path from A to B is found
    nResult = itDL->second;

    // write nodes into pOutBuffer
    for (int i=0; i<nResult; ++i)
    {
      nCurrX = std::get<0>(itDL->first);
      nCurrY = std::get<1>(itDL->first);

      pOutBuffer[i] = nCurrY*nMapHeight+nCurrX;

      auto min = itDL;

      // find the lowest adjacent node value
      for (int j=0; j<4; ++j)
      {
        nAdjX = nCurrX+DIR[j].first;
        nAdjY = nCurrY+DIR[j].second;

        itDL = DistanceMap.find(Node(nAdjX,nAdjY));
        
        // check if record exists
        if (itDL != DistanceMap.end())
        {
          if (itDL->second < min->second) min = itDL;
        }
      }
      itDL = min;
    }
  }
  else
  {
    // failed to find the path
    nResult = -1;
  }

  return nResult;
}




