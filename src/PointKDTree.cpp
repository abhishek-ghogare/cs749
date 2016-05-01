#include "PointKDTree.hpp"
#include <cfloat>

PointKDTree::Node::~Node()
{
	delete lo;
	delete hi;
}

PointKDTree::PointKDTree(std::vector<Point> const & points)
: root(NULL)
{

	// A kd-tree is just a binary search tree, and is constructed in a near-identical way.
	//
	// - Initially assign (pointers to) all points to the root node.
	// - Recursively subdivide the points, splitting the parent box in half along the longest axis and creating two child nodes
	//   for each split. Stop when number of points in node <= MAX_POINTS_PER_LEAF.
	// - Don't forget to save space by clearing the arrays of points in internal nodes. Only the leaves need to store references
	//   to points.

	std::vector<const Point *> point_ptrs;
	for ( std::vector<Point>::const_iterator it = points.begin() ; it != points.end(); ++it )
	{
		const Point *n = &points[it-points.begin()];
		point_ptrs.push_back( n );
	}

	minDistance = 100000;
	maxDistance = 0;
	root = GenKDTreeRec(point_ptrs);
	minDistance = sqrt(minDistance);
	maxDistance = sqrt(maxDistance);
	//minDistance = minDistance<0.01 ? 0.01 : minDistance;
}

/** Recursively generate KD tree */
PointKDTree::Node * PointKDTree::GenKDTreeRec ( std::vector<const Point *> const & points )
{
	static size_t const MAX_POINTS_PER_LEAF = 10;

	Node * node = new Node();

	// Add points to self point list
	node->points = std::vector<const Point *>(points);
	// Compute bounding box
	for ( std::vector<const Point *>::const_iterator it = points.begin() ; it != points.end(); ++it )
	{
		node->bbox.addPoint((*it)->getPosition());
	}

	// Leaf node, just return
	if(points.size()<=MAX_POINTS_PER_LEAF)
	{
		DGP_CONSOLE << "Leaf node with " << points.size() << " points";

		// Lets calculate minimum distance
		for ( uint i=0 ; i<points.size() ; ++i )
		{
			for (  uint j=i+1 ; j<points.size() ; ++j )
			{
				Vector3 d = node->bbox.getHigh() - node->bbox.getLow();

				Real maxlen = fmax(d.x(),fmax(d.y(),d.z()));

				if      (maxlen > maxDistance) maxDistance = maxlen;
				else if (maxlen < minDistance) minDistance = maxlen;
			}
		}
		return node;	
	}
	
	// Calculate along the axis length of bounding box
	Real xlen = node->bbox.getHigh().x() - node->bbox.getLow().x();
	Real ylen = node->bbox.getHigh().y() - node->bbox.getLow().y();
	Real zlen = node->bbox.getHigh().z() - node->bbox.getLow().z();

	DGP_CONSOLE << "\nBounding box: " << node->bbox.getLow() << node->bbox.getHigh();
	DGP_CONSOLE << "xlen: " << xlen ;
	DGP_CONSOLE << "ylen: " << ylen ;
	DGP_CONSOLE << "zlen: " << zlen ;
	DGP_CONSOLE << "#of points: " << points.size() ;

	// Lets find median across longest axis, based on which partition will be made
	std::vector<Real> numbers;
	Real median;
	int axis = -1;
	std::vector<const Point *> lo, hi;
	Real x=xlen, y=ylen, z=zlen;
	
	while(1)
	{
		hi.clear();
		lo.clear();
		numbers.clear();

		if 		( xlen >= ylen && xlen >= zlen )	{axis = 0;}//median=(node->bbox.getLow().x() + xlen/2);} // Divide along x axis
		else if ( ylen >= xlen && ylen >= zlen )	{axis = 1;}//median=(node->bbox.getLow().y() + ylen/2);} // Divide along y axis
		else if ( zlen >= xlen && zlen >= ylen )	{axis = 2;}//median=(node->bbox.getLow().z() + zlen/2);} // Divide along z axis
		else	{DGP_CONSOLE << "Something is not right"; exit (1);}

		for ( std::vector<const Point *>::const_iterator it = points.begin() ; it != points.end(); ++it )
		{
			numbers.push_back((*it)->getPosition()[axis]);
		}
		std::sort(numbers.begin(), numbers.end());
		median = numbers[numbers.size()/2];

		// Now divide points based on axis and median
		for ( std::vector<const Point *>::const_iterator it = points.begin() ; it != points.end(); ++it )
		{
			if ( (*it)->getPosition()[axis] >= median )
			{
				hi.push_back(*it);
				//DGP_CONSOLE << "adding to hi" ;
			}
			else //if( (*it)->getPosition()[axis] < median )
			{
				lo.push_back(*it);
				//DGP_CONSOLE << "adding to lo" ;
			}
		}

		if ( xlen==0 && ylen==0 && zlen==0 )
		{
			DGP_CONSOLE << "all zero, old was "<<x<<","<<y<<","<<z;
			for ( std::vector<const Point *>::const_iterator it = points.begin() ; it != points.end(); ++it )
			{
				DGP_CONSOLE << (*it)->getPosition().toString();
			}
			exit(2);
		}

		// Check if one of the partition is empty, so that we can patition along next longest axis instead
		if (hi.size()==0 || lo.size()==0)
		{
			// So repeat partition on another axis
			// zeroing the axis len, so that next longest axis would be chosen in next iteration
			if 		(axis==0)	xlen=0;
			else if (axis==1)	ylen=0;
			else if (axis==2)	zlen=0;
			else	{DGP_CONSOLE << "Something is not right"; exit (1);}

			DGP_CONSOLE << "::::::::::::One the partition is empty";
			DGP_CONSOLE << hi.size() << " " << lo.size();
			DGP_CONSOLE << "xlen: " << xlen ;
			DGP_CONSOLE << "ylen: " << ylen ;
			DGP_CONSOLE << "zlen: " << zlen ;
			DGP_CONSOLE << "axis: " << axis ;
			DGP_CONSOLE << "medi: " << median ;
			axis = -1;
		}
		else
		{
			// Everything is all right
			break;
		}
	}

	DGP_CONSOLE << axis << "-axis:median:" << median;	

	node->lo = GenKDTreeRec(lo);
	node->hi = GenKDTreeRec(hi);

	// clear points as non-leaf node
	node->points.clear();
	return node;
}

//PointKDTree::rangeQuery(AxisAlignedBox3 &,             std::vector<Point*>&)pkdt.rangeQuery(bbox, pts);
void
PointKDTree::rangeQuery(AxisAlignedBox3 const & query, std::vector<const Point *> & points_in_range) const
{
	// Write a helper function rangeQuery(node, query, points_in_range):
	//   - If node->bbox does not intersect query, return
	//   - If node->lo && node->lo->bbox intersects query, rangeQuery(node->lo, query, points_in_range)
	//   - If node->hi && node->hi->bbox intersects query, rangeQuery(node->hi, query, points_in_range)
	rangeQueryRec(root, query, points_in_range);
}

void PointKDTree::rangeQueryRec(PointKDTree::Node * node, AxisAlignedBox3 const & query, std::vector<const Point *> & points_in_range) const
{
	if(node->bbox.intersects(query))
	{
		// Both children will be either NULL or non-NULL at the same time
		if (!node->lo && !node->hi)
		{
			// If leaf node
			for ( std::vector<const Point *>::const_iterator it = node->points.begin() ; it != node->points.end(); ++it )
			{
				if ( query.intersects((*it)->getPosition()) )
					points_in_range.push_back(*it);
			}
		}
		else
		{
			if (node->lo)	rangeQueryRec(node->lo, query, points_in_range);
			if (node->hi)	rangeQueryRec(node->hi, query, points_in_range);
		}
	}
}


