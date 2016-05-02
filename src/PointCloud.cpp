#include "PointCloud.hpp"
#include "DGP/Graphics/Shader.hpp"
#include <fstream>
#include <sstream>

PointCloud::PointCloud(std::vector<Point> const & points_)
: points(points_)
{
  recomputeAABB();
}

PointCloud::PointCloud(std::vector<Vector3> const & positions, std::vector<Vector3> const & normals)
{
  alwaysAssertM(positions.size() == normals.size(), "PointCloud: Number of positions != number of normals");

  for (size_t i = 0; i < positions.size(); ++i)
	points.push_back(Point(positions[i], normals[i]));

  recomputeAABB();
}

bool
PointCloud::load(std::string const & path)
{
  // Simple format: Each line is either
  //   x y z
  //    OR
  //   x y z nx ny nz
  //
  // where (nx, ny, nz) is the normal

  std::ifstream in(path.c_str());
  if (!in)
  {
	DGP_ERROR << "Could not open file for reading: " << path;
	return false;
  }

  std::string line;
  while (getline(in, line))
  {
	// Skip empty lines
	line = trimWhitespace(line);
	if (line.empty())
	  continue;

	std::istringstream line_in(line);
	Vector3 p;
	if (!(line_in >> p[0] >> p[1] >> p[2]))
	{
	  DGP_ERROR << "Could not read point " << points.size() << " from line: " << line;
	  return false;
	}

	// Normal is optional
	Vector3 n;
	if (!(line_in >> n[0] >> n[1] >> n[2]))  // doesn't properly handle malformed lines, but we'll ignore this for now
	  n = Vector3::zero();

	points.push_back(Point(p, n));
  }

  recomputeAABB();

  return true;
}

bool
PointCloud::save(std::string const & path) const
{
  std::ofstream out(path.c_str(), std::ios::binary);
  if (!out)
  {
	DGP_ERROR << "Could not open file for writing: " << path;
	return false;
  }

  for (size_t i = 0; i < points.size(); ++i)
  {
	Vector3 const & p = points[i].getPosition();
	Vector3 const & n = points[i].getNormal();
	out << p[0] << ' ' << p[1] << ' ' << p[2] << ' ' << n[0] << ' ' << n[1] << ' ' << n[2] << '\n';
  }

  return true;
}

Graphics::Shader *
createPointShader(Graphics::RenderSystem & rs)
{
  static std::string const VERTEX_SHADER =
"void main()\n"
"{\n"
"  gl_Position = ftransform();\n"
"  gl_FrontColor = gl_Color;\n"
"  gl_BackColor = gl_Color;\n"
"}\n";

  static std::string const FRAGMENT_SHADER =
"void main()\n"
"{\n"
"  gl_FragColor = gl_Color;\n"
"}\n";

  Graphics::Shader * shader = rs.createShader("Point Graphics::Shader");
  if (!shader)
	throw Error("Could not create point shader");

  // Will throw errors on failure
  shader->attachModuleFromString(Graphics::Shader::ModuleType::VERTEX, VERTEX_SHADER.c_str());
  shader->attachModuleFromString(Graphics::Shader::ModuleType::FRAGMENT, FRAGMENT_SHADER.c_str());

  return shader;
}

void
PointCloud::draw(Graphics::RenderSystem & rs, Real normal_len, ColorRGBA const & color) const
{
  // Make this static to ensure just one shader is created. Assumes rendersystem is constant, not the best design pattern.
  static Graphics::Shader * shader = createPointShader(rs);

  rs.pushShader();
  rs.pushColorFlags();
  rs.pushShapeFlags();

	rs.setShader(shader);
	rs.setColor(color);  // dark grey
	rs.setPointSize(2.0f);

	rs.beginPrimitive(Graphics::RenderSystem::Primitive::POINTS);
	  for (size_t i = 0; i < points.size(); ++i)
		rs.sendVertex(points[i].getPosition());
	rs.endPrimitive();

	if (normal_len > 0)
	{
	  rs.setColor(ColorRGB(0.5, 0.5, 1.0));  // blue

	  rs.beginPrimitive(Graphics::RenderSystem::Primitive::LINES);
		for (size_t i = 0; i < points.size(); ++i)
		{
		  Vector3 const & p = points[i].getPosition();
		  Vector3 const & n = points[i].getNormal();

		  rs.sendVertex(p);
		  rs.sendVertex(p + normal_len * n);
		}
	  rs.endPrimitive();
	}

  rs.popShapeFlags();
  rs.popColorFlags();
  rs.popShader();
}

void
PointCloud::recomputeAABB()
{
  bbox.setNull();

  for (size_t i = 0; i < points.size(); ++i)
	bbox.merge(points[i].getPosition());
}

void
PointCloud::estimateNormals()
{
	PointKDTree pkdt(points);

	Real d = pkdt.minDistance/2;
	Vector3 bbox_diff(d,d,d);
	//Vector3 bbox_diff(0.5,0.5,0.5);

	DGP_CONSOLE << "minDistance : " << pkdt.minDistance;
	DGP_CONSOLE << "diff box : " << bbox_diff;

	for(auto & point: points)
	{
		std::vector<const Point *> neighbours;	// TODO:: check if clearing required
		neighbours.clear();
		AxisAlignedBox3 bbox( point.getPosition()-bbox_diff , point.getPosition()+bbox_diff );
		pkdt.rangeQuery(bbox, neighbours);
		DGP_CONSOLE << "\nCurent Point : " << point.getPosition();
		DGP_CONSOLE << "BBox : " << bbox.getLow() << bbox.getHigh();
		DGP_CONSOLE << "Neighbours : ";
		for(auto const& np: neighbours)
		{
			DGP_CONSOLE << np->getPosition();
		}

		MatrixMN<3, 1, Real> a, xi;
		Matrix3 mat;

		a.makeZero();
		for(auto const& np: neighbours)
		{
			xi.setColumn(0,np->getPosition());
			a += xi;
		}
		a/=neighbours.size();
		DGP_CONSOLE << "Value of a : " << a;

		mat.makeZero();
		for(auto const& np: neighbours)
		{
			xi.setColumn(0,np->getPosition());
			mat += (xi-a)*((xi-a).transpose());
		}
		DGP_CONSOLE << "Value of matrix : " << mat;

		Real eigenVal[3];
		VectorN<3, Real> eigenVec[3];
		mat.eigenSolveSymmetric(eigenVal, eigenVec);

		DGP_CONSOLE << "Eigen values : " << eigenVal[0] << " " << eigenVal[1] << " " << eigenVal[2];
		DGP_CONSOLE << "Eigen vector : " << eigenVec[0] << " " << eigenVec[1] << " " << eigenVec[2];

		// Index of smallest eigenvalue
		int i = eigenVal[0]<eigenVal[1] ? (eigenVal[0]<eigenVal[2]?0:2) : (eigenVal[1]<eigenVal[2]?1:2);
		DGP_CONSOLE << i<< "Normal : " << eigenVec[i];
		point.setNormal(eigenVec[i].unit());
	}
/*
	Vector3 lo(0.4,0.4,0.4), hi(0.5,2,0.5);
	AxisAlignedBox3 bbox(lo,hi);
	std::vector<const Point *> pts;

	pkdt.rangeQuery(bbox, pts);

	DGP_CONSOLE << "\nPoints inside bounding box";
	for(auto const& point: pts)
	{
		DGP_CONSOLE << point->getPosition();
	}

	MatrixMN<3, 1, Real> a, xi;
	Matrix3 mat;

	a.makeZero();
	for(auto const& point: pts)
	{
		xi.setColumn(0,point->getPosition());
		a += xi;
	}
	a/=pts.size();
	DGP_CONSOLE << a;

	mat.makeZero();
	for(auto const& point: pts)
	{
		xi.setColumn(0,point->getPosition());
		mat += (xi-a)*((xi-a).transpose());
	}
	DGP_CONSOLE<<mat;

	Real eigenVal[3];
	VectorN<3, Real> eigenVec[3];
	mat.eigenSolveSymmetric(eigenVal, eigenVec);

	DGP_CONSOLE << "\nEigen vectors and eigen values";
	DGP_CONSOLE << eigenVal[0] << " " << eigenVal[1] << " " << eigenVal[2];
	DGP_CONSOLE << eigenVec[0] << " " << eigenVec[1] << " " << eigenVec[2];
*/

/*
	DGP_CONSOLE << "\nPoints in box: "<< lo << " " << hi;
	for ( std::vector<const Point *>::const_iterator it = pts.begin() ; it != pts.end(); ++it )
	{
		DGP_CONSOLE << (*it)->getPosition();
	}


	DGP_CONSOLE << "\nActual points: "<< lo << " " << hi;
	for ( std::vector<Point>::iterator it = points.begin() ; it != points.end(); ++it )
	{
		if(	it->getPosition().x()<= hi.x() &&
			it->getPosition().x()>= lo.x() &&
			it->getPosition().y()<= hi.y() &&
			it->getPosition().y()>= lo.y() &&
			it->getPosition().z()<= hi.z() &&
			it->getPosition().z()>= lo.z()
			)
		{
			DGP_CONSOLE << it->getPosition();
		}
	}*/
}

void
PointCloud::adaptiveDownsample()
{
  // TODO
}


bool
PointCloud::loadISM_BIN(std::string const & path_)
{
  clear();

  // Do loading
  BinaryInputStream in(path_, Endianness::LITTLE);

  nlabels = in.readInt64();
  nobjects = in.readInt64();
  long npoints = in.readInt64();

  DGP_CONSOLE << "PointCloud: '" << path_ << "' has " << npoints << " points, " << nlabels << " labels and " << nobjects
               << " objects";

  points.resize((size_t)npoints);
  int num_features = 0;

  for (int i = 0; i < points.size(); ++i)
  {
    Point & p = points[i];

    p.label_index = in.readInt64();
    p.object_index = in.readInt64();

    p.position[0] = in.readFloat32();
    p.position[1] = in.readFloat32();
    p.position[2] = in.readFloat32();

    p.normal[0] = in.readFloat32();
    p.normal[1] = in.readFloat32();
    p.normal[2] = in.readFloat32();

    Real height = in.readFloat32();
    Real f12 = in.readFloat32();
    Real f13 = in.readFloat32();
    Real f23 = in.readFloat32();

    int nfeatures = 4 + in.readInt32();  // 4 for height and three covariance ratios
    if (i == 0)
    {
      num_features = nfeatures;
    }
    else if (num_features != nfeatures)
    {
      DGP_ERROR << "PointCloud: Inconsistent number of features in " << path_;
      return false;
    }

    p.features.resize(nfeatures);

    p.features[0] = height;
    p.features[1] = f12;
    p.features[2] = f13;
    p.features[3] = f23;

    for (int j = 4; j < nfeatures; ++j)
      p.features[j] = in.readFloat32();
  }

  return true;
}


bool
PointCloud::extract_objects(std::string const & out_dir_path)
{
	std::vector<PointCloud> objects(nobjects);


	DGP_CONSOLE << "Extracting "<< nobjects << " objects from point cloud" ;

	for ( int i=0 ; i<points.size() ; i++) {
		objects[int(points[i].object_index)].addPoint(points[i]);
	}

	DGP_CONSOLE << "Objects extracted from point cloud, now saving to files" ;
	for ( int i=0 ; i<nobjects ; i++ ) {
		std::ostringstream oss;
		oss << out_dir_path << "/" << i << ".pts";
		objects[i].save(oss.str());
	}

	DGP_CONSOLE << "Objects saved to "<< out_dir_path << " directory" ;
}
