#pragma once
#define _USE_MATH_DEFINES

#include <math.h>
struct Rect
{
	short _blX,_blY,_trX,_trY;
	short cx,cy;
	Rect (short __blX,short __blY,short __trX,short __trY) : _blX(__blX),_blY(__blY),_trX(__trX),_trY(__trY)
	{

	}

	int pixDist(Rect * rhs)
	{
		int boundaryDistHoriz = min(min(min(abs(_blX-rhs->_trX),abs(_trX-rhs->_blX)),abs(_blX-rhs->_blX)),abs(_trX-rhs->_trX)) ;
		int boundaryDistVert = min(min(min(abs(_trY-rhs->_blY),abs(_blY-rhs->_trY)),abs(_blY-rhs->_blY)),abs(_trY-rhs->_trY)) ;
		int boundaryDist = max(boundaryDistHoriz,boundaryDistVert);
		return boundaryDist;
	}
	void expand(int lx,int rx,int up, int down)
	{
		_blX-=lx;
		_trX+=rx;
		_trY-=up;
		_blY+=down;
	}
	bool isColliding(Rect * rhs)
	{
		if (_blX>rhs->_trX || _trX<rhs->_blX || 
			_blY<rhs->_trY || _trY>rhs->_blY )
			return false;
		return true;
	}
	int getArea()
	{
		return (_trX-_blX)*(_blY-_trY);
	}
};