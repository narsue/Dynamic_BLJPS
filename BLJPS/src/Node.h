#pragma once
#include <vector>
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))
struct Coordinate
{
	short x,y;
	Coordinate()
	{
	}
	Coordinate(short _x,short _y) : x(_x),y(_y)
	{
	}
	float dist(const Coordinate& rhs)
	{
		int absX =abs(x-rhs.x);
		int absY =abs(y-rhs.y);

		int diagDist = min(absX,absY);
		int straightDist = max(absX,absY)-diagDist;
		return diagDist*1.414213562373095f+straightDist;
	}
	float distSqrt(const Coordinate& rhs)
	{
		int absX =abs(x-rhs.x);
		int absY =abs(y-rhs.y);

		int diagDist = min(absX,absY);
		int straightDist = max(absX,absY)-diagDist;
		return diagDist*1.414213562373095f+straightDist;
	}
	void add(const Coordinate& rhs)
	{
		x+=rhs.x;
		y+=rhs.y;
	}

};
struct Node
{
	float score,total;
	Coordinate pos;
	Node*parent;
	char dir;
	Node()
	{

	}
	static float estimateDistance (const Coordinate &start, const Coordinate& end)
	{
		int absX =abs(start.x-end.x);
		int absY =abs(start.y-end.y);

		int diagDist = min(absX,absY);
		int straightDist = max(absX,absY)-diagDist;
		return diagDist*1.414213562373095f+straightDist;
		//return sqrt((float)(pos.x-end.x)*(pos.x-end.x)+(pos.y-end.y)*(pos.y-end.y));//max (abs (start.x - end.x), abs (start.y - end.y));
	}
	void reset(const Coordinate& _pos, Node*_parent,const int eX,const int eY,const char _dir)
	{
		pos =_pos;
		parent = _parent;
		dir = _dir;
		if (parent)
			score=estimateDistance(pos,parent->pos)+parent->score;
		else
			score=0;
		
		total=score + estimateDistance(pos,Coordinate(eX,eY));//sqrt((float)(pos.x-eX)*(pos.x-eX)+(pos.y-eY)*(pos.y-eY));
	}
};
class CompareNode
{
public:
	bool operator() (const Node* t1, const Node* t2) const// Returns true if t1 is earlier than t2
    {
		if (t1->total<=t2->total)//t1->cost+
			return false;
		return true;
    }
};
#define NODE_CONTAINER_BUCKET_SIZE 256
struct NodeContainer
{
	NodeContainer()
	{
		numDistributedNodes=bucketId=nodeId=0;
		//for (int i =0;i<3;i++)
			bucket.push_back(new Node[NODE_CONTAINER_BUCKET_SIZE]());
	}
	~NodeContainer()
	{
		for (unsigned int i=0;i<bucket.size();i++)
			delete [] bucket[i];
	}
	Node* getNewNode(const Coordinate & _pos,int _endX,int _endY,char dir,Node* _parent)
	{
		if (bucketId>bucket.size()-1)
			bucket.push_back(new Node[NODE_CONTAINER_BUCKET_SIZE]());
		Node* ret =  &bucket[bucketId][nodeId];
		ret->reset(_pos,_parent,_endX,_endY,dir);
		numDistributedNodes++;
		nodeId++;
		if (nodeId==NODE_CONTAINER_BUCKET_SIZE)
		{
			nodeId=0;
			bucketId++;
		}
		return ret;
	}
	void reset()
	{
		numDistributedNodes=bucketId=nodeId=0;
		//printf("BucketSize:%d\n",bucket.size());
	}
	void deleteLastNode()
	{
		numDistributedNodes--;
		nodeId--;
		if (nodeId<0)
		{
			nodeId=NODE_CONTAINER_BUCKET_SIZE-1;
			bucketId--;
		}
	}
	unsigned int numDistributedNodes,bucketId,nodeId;
	std::vector<Node*> bucket;
};
float pathLength(std::vector<Coordinate> & solutionPath)
{
	if (solutionPath.size() == 0)
		return 0;
	float dist = 0;
	for (int i = 0; i<solutionPath.size() - 1; i++)
		dist += solutionPath[i].distSqrt(solutionPath[i + 1]);
	return dist;
}
bool nonSimilarFloat(float l, float r)
{
	return abs(1.0f - l / r) > 0.01;
}