
#include <map>
#include <deque>
#include <vector>
#include <thread>
#include <algorithm>

typedef std::pair<int, int> Node;


class cPathfinderWorker
{
  public:
    cPathfinderWorker();
    ~cPathfinderWorker();

    void UseMultiThread(bool _bMultithread, std::vector<Node>* _vecPastManhattan,
                        bool* _bKeepSearching);

    void UseManhattanBBox(bool _bUseManhattanBbox, unsigned int _nMDWindowSize);

    
    // returns path legth if it is found or -1 on failure
    int FindPath(const int nStartX, const int nStartY,
                const int nTargetX, const int nTargetY,
                const unsigned char* pMap,
                const int nMapWidth, const int nMapHeight,
                int* pOutBuffer, const int nOutBufferSize);

    // reconstructs partial or full paths' on current DistanceMap
    int Reconstruct(const int nTargetX, const int nTargetY, int* pOutBuffer);    
    
    // Control the state of the worker
    void stopSearching(){*bKeepSearching = false;}    
    bool isSearching(){return *bKeepSearching;}
    
  private:
    // internal function that initiates the seach
    bool Search(const int nStartX, const int nStartY,
                const int nTargetX, const int nTargetY,
                const unsigned char* pMap,
                const int nMapWidth, const int nMapHeight,
                const int nOutBufferSize);

    // variables to hold intermediate values during the searches
    unsigned int nCurrX = 0;
    unsigned int nCurrY = 0;
    unsigned int nAdjX  = 0;
    unsigned int nAdjY  = 0;
    unsigned int nMDAdjEnd  = 0;
    unsigned int nMDCurrEnd = 0;
    
    int nMapWidth;
    int nMapHeight;
    
    bool bPathFound = false;
    bool* bKeepSearching;

    
    bool bUseMultipleThreads = false;
    std::vector<Node>* vecPastManhattan;

    bool bUseManhattanBbox = false;
    unsigned int nMDWindowSize = 3;

    
    // Holds the information about the distance from start to node
    std::map<Node, unsigned int> DistanceMap;
    
    // Four allowed directions to move
    std::pair<char,char> DIR[4] = { std::make_pair( 1, 0),
                                    std::make_pair(-1, 0),
                                    std::make_pair( 0, 1),
                                    std::make_pair( 0,-1)};
};

class cPathfinder
{  
  public:
    cPathfinder(bool _bUseMultipleThreads,
                bool _bUseManhattanBbox);
    
    // returns path legth if it is found or -1 on failure
    int FindPath(const int nStartX, const int nStartY,
                 const int nTargetX, const int nTargetY,
                 const unsigned char* pMap,
                 const int nMapWidth, const int nMapHeight,
                 int* pOutBuffer, const int nOutBufferSize);
    
  private:    
    int  nResult;              // Variable to hold the result of execution
    bool bKeepSearching;
    
    bool bUseMultipleThreads;
    bool bUseManhattanBbox;
};