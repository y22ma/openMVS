/*
 * PoissonMesh.cpp
 *
 * Copyright (c) 2014-2015 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include <boost/program_options.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <string>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("PoissonMesh")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
String strDenseConfigFileName;
unsigned nArchiveType;
unsigned nOctreeDepth;
unsigned nIsoDepth;
unsigned nSamplesPerNode;
bool bReverseNormals;
float fNormalRadius;
int nProcessPriority;
unsigned nMaxThreads;
String strConfigFileName;
boost::program_options::variables_map vm;
} // namespace OPT

// initialize and parse the command line parameters
bool Initialize(size_t argc, LPCTSTR* argv)
{
  // initialize log and console
  OPEN_LOG();
  OPEN_LOGCONSOLE();

  // group of options allowed only on command line
  boost::program_options::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "produce this help message")
    ("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
    ("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
    ("archive-type", boost::program_options::value<unsigned>(&OPT::nArchiveType)->default_value(2), "project archive type: 0-text, 1-binary, 2-compressed binary")
    ("process-priority", boost::program_options::value<int>(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
    ("max-threads", boost::program_options::value<unsigned>(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
    ("octree-depth", boost::program_options::value<unsigned>(&OPT::nOctreeDepth)->default_value(12), "octree depth")
    ("iso-depth", boost::program_options::value<unsigned>(&OPT::nIsoDepth)->default_value(8), "iso depth")
    ("iso-depth", boost::program_options::value<unsigned>(&OPT::nSamplesPerNode)->default_value(8), "minimum samples per node for octree")
    ("normal-radius", boost::program_options::value<float>(&OPT::fNormalRadius)->default_value(0.5), "radius of sphere for including points for normal estimation")
    ("reverse-normals", boost::program_options::value<bool>(&OPT::bReverseNormals)->default_value(true), "flag to reverse normals")
#if TD_VERBOSE != TD_VERBOSE_OFF
    ("verbosity,v", boost::program_options::value<int>(&g_nVerbosityLevel)->default_value(
#if TD_VERBOSE == TD_VERBOSE_DEBUG
      3
#else
      2
#endif
      ), "verbosity level")
#endif
    ;
  
  // group of options allowed both on command line and in config file
  boost::program_options::options_description config("Scene Split Options");
  config.add_options()
        ("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list");
  
  boost::program_options::options_description cmdline_options;
  cmdline_options.add(generic).add(config);
  
  boost::program_options::options_description config_file_options;
  config_file_options.add(config);
  
  boost::program_options::positional_options_description p;
  p.add("input-file", -1);
  
  try {
  	// parse command line options
  	boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
  	boost::program_options::notify(OPT::vm);
  	INIT_WORKING_FOLDER;
  	// parse configuration file
  	std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName));
  	if (ifs) {
  		boost::program_options::store(parse_config_file(ifs, config_file_options), OPT::vm);
  		boost::program_options::notify(OPT::vm);
  	}
  }
  catch (const std::exception& e) {
  	LOG(e.what());
  	return false;
  }
  
  // initialize the log file
  OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());
  
  // print application details: version and command line
  Util::LogBuild();
  LOG(_T("Command line:%s"), Util::CommandLineToString(argc, argv).c_str());
  
  // validate input
  Util::ensureValidPath(OPT::strInputFileName);
  Util::ensureUnifySlash(OPT::strInputFileName);
  if (OPT::vm.count("help") || OPT::strInputFileName.IsEmpty()) {
    boost::program_options::options_description visible("Available options");
    visible.add(generic).add(config);
    GET_LOG() << visible;
  }
  if (OPT::strInputFileName.IsEmpty())
    return false;
  
  // initialize optional options
  Util::ensureValidPath(OPT::strOutputFileName);
  Util::ensureUnifySlash(OPT::strOutputFileName);
  if (OPT::strOutputFileName.IsEmpty())
    OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + _T(".mvs");
  
  #ifdef _USE_BREAKPAD
  // start memory dumper
  MiniDumper::Create(APPNAME, WORKING_FOLDER);
  #endif
  return true;
}

// finalize application instance
void Finalize()
{
#if TD_VERBOSE != TD_VERBOSE_OFF
  // print memory statistics
  Util::LogMemoryInfo();
#endif
  
  CLOSE_LOGCONSOLE();
  CLOSE_LOG();
}



int main(int argc, LPCTSTR* argv)
{
#ifdef _DEBUGINFO
  // set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
#endif
  
  if (!Initialize(argc, argv))
  {
    return EXIT_FAILURE;
  }
  
  Scene scene(OPT::nMaxThreads);
  // load and estimate a dense point-cloud
  if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
  {
    return EXIT_FAILURE;
  }
  if (scene.pointcloud.IsEmpty()) {
    VERBOSE("error: empty initial point-cloud");
    return EXIT_FAILURE;
  }
  TD_TIMER_START();

  // convert to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZ>);
  ptcloud->width = scene.pointcloud.points.size();
  ptcloud->height = 1;
  ptcloud->points.resize(ptcloud->width);
  for (uint32_t i = 0; i < scene.pointcloud.points.size(); i++)
  {
    ptcloud->points[i].x = scene.pointcloud.points[i][0];
    ptcloud->points[i].y = scene.pointcloud.points[i][1];
    ptcloud->points[i].z = scene.pointcloud.points[i][2];
  }

  std::cout << "begin normal estimation" << std::endl;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(ptcloud);
  ne.setRadiusSearch(OPT::fNormalRadius);
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*ptcloud, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);
  std::cout << "normal estimation complete" << std::endl;
  std::cout << "reverse normals' direction" << std::endl;

  if (OPT::bReverseNormals)
  {
    for(size_t i = 0; i < cloud_normals->size(); ++i){
      cloud_normals->points[i].normal_x *= -1;
      cloud_normals->points[i].normal_y *= -1;
      cloud_normals->points[i].normal_z *= -1;
    }
  }

  std::cout << "combine points and normals" << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*ptcloud, *cloud_normals, *cloud_smoothed_normals);

  pcl::PLYWriter writer;
  const String baseFileName(MAKE_PATH_SAFE(Util::getFullFileName(OPT::strOutputFileName) +
      "_poisson"));
  writer.write(baseFileName+_T(".ply"), *cloud_smoothed_normals);

  std::cout << "begin poisson reconstruction" << std::endl;
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(OPT::nOctreeDepth);
  poisson.setIsoDivide(OPT::nIsoDepth);
  poisson.setSamplesPerNode(OPT::nSamplesPerNode);
  poisson.setInputCloud(cloud_smoothed_normals);
  pcl::PolygonMesh mesh;
  poisson.reconstruct(mesh);

  // save the final mesh
  pcl::io::savePLYFile(baseFileName+_T(".ply"), mesh);

  Finalize();
  return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
