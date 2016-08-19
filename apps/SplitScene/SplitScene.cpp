/*
 * SplitScene.cpp
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
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <string>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("SplitScene")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
String strDenseConfigFileName;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
String strConfigFileName;
boost::program_options::variables_map vm;
unsigned gridWidth;
unsigned gridHeight;
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
        ("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
  	("grid-width", boost::program_options::value<unsigned>(&OPT::gridWidth), "grid width")
  	("grid-height", boost::program_options::value<unsigned>(&OPT::gridHeight), "grid height");
  
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

  // split pointcloud here
  // create sub scenes
  uint32_t gridSize = OPT::gridWidth*OPT::gridHeight; 
  std::shared_ptr<Scene> subScenes[OPT::gridWidth*OPT::gridHeight];
  for (uint32_t i = 0; i < gridSize; i++)
  {
    subScenes[i].reset(new Scene(scene));
    subScenes[i]->pointcloud.Release();
  }

  // convert to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZ>);
  ptcloud->width = scene.pointcloud.points.size();
  ptcloud->height = 1;
  ptcloud->points.resize(ptcloud->width);
  double minX, minY, minZ;
  double maxX, maxY, maxZ;
  minX = minY = minZ = 100000;
  maxX = maxY = maxZ = -100000;
  for (uint32_t i = 0; i < scene.pointcloud.points.size(); i++)
  {
    ptcloud->points[i].x = scene.pointcloud.points[i][0];
    ptcloud->points[i].y = scene.pointcloud.points[i][1];
    ptcloud->points[i].z = scene.pointcloud.points[i][2];
    if (ptcloud->points[i].x <= minX)
    {
      minX = ptcloud->points[i].x;
    }
    if (ptcloud->points[i].x >= maxX)
    {
      maxX = ptcloud->points[i].x;
    } 

    if (ptcloud->points[i].y <= minY)
    {
      minY = ptcloud->points[i].y;
    }
    if (ptcloud->points[i].y >= maxY)
    {
      maxY = ptcloud->points[i].y;
    } 

    if (ptcloud->points[i].z <= minZ)
    {
      minZ = ptcloud->points[i].z;
    }
    if (ptcloud->points[i].z >= maxZ)
    {
      maxZ = ptcloud->points[i].z;
    } 
  }
  //pcl::PLYWriter writer;
  //writer.write("/tmp/pcl_scene.ply", *ptcloud);

  // getting bounds
  std::cout << minX << " " << minY << " " << minZ << std::endl;
  std::cout << maxX << " " << maxY << " " << maxZ << std::endl;
  float tileWidth  = (maxX - minX)/OPT::gridWidth;
  float tileHeight = (maxY - minY)/OPT::gridHeight;
  std::cout << tileWidth << std::endl;
  std::cout << tileHeight << std::endl;
  for (uint32_t i = 0; i < OPT::gridHeight; i++)
  {
    for (uint32_t j = 0; j < OPT::gridWidth; j++)
    {
      pcl::PassThrough<pcl::PointXYZ> ptfilter(true);
      ptfilter.setInputCloud(ptcloud);
      
      double tileXMin = minX + tileWidth*j;	
      double tileYMin = minY + tileHeight*i;	
      double tileXMax = minX + tileWidth*(j + 1);	
      double tileYMax = minY + tileHeight*(i + 1);	
      
      boost::shared_ptr<std::vector<int> > indices_x(new std::vector<int>());
      ptfilter.setFilterFieldName("x");
      ptfilter.setFilterLimits(tileXMin, tileXMax);
      ptfilter.filter(*indices_x);
      
      boost::shared_ptr<std::vector<int> > indices_xy(new std::vector<int>());
      ptfilter.setIndices (indices_x);
      ptfilter.setFilterFieldName("y");
      ptfilter.setFilterLimits(tileYMin, tileYMax);
      ptfilter.filter(*indices_xy);
      
      uint32_t sIdx = i*OPT::gridWidth + j;
      subScenes[sIdx]->pointcloud.points.Resize(indices_xy->size()); 
      if (!scene.pointcloud.colors.IsEmpty())
      {
        subScenes[sIdx]->pointcloud.colors.Resize(indices_xy->size()); 
      }
      if (!scene.pointcloud.normals.IsEmpty())
      {
        subScenes[sIdx]->pointcloud.normals.Resize(indices_xy->size()); 
      }
      if (!scene.pointcloud.pointWeights.IsEmpty())
      {
        subScenes[sIdx]->pointcloud.pointWeights.Resize(indices_xy->size()); 
      }
      if (!scene.pointcloud.pointViews.IsEmpty())
      {
        subScenes[sIdx]->pointcloud.pointViews.Resize(indices_xy->size()); 
      }
      for (uint32_t k = 0; k < indices_xy->size(); k++)
      {
        uint32_t idx = (*indices_xy)[k];
        subScenes[sIdx]->pointcloud.points[k] = scene.pointcloud.points[idx];
        if (!scene.pointcloud.colors.IsEmpty())
        {
          subScenes[sIdx]->pointcloud.colors[k] = scene.pointcloud.colors[idx];
        }
        if (!scene.pointcloud.normals.IsEmpty())
        {
          subScenes[sIdx]->pointcloud.normals[k] = scene.pointcloud.normals[idx];
        }
        if (!scene.pointcloud.pointViews.IsEmpty())
        {
          subScenes[sIdx]->pointcloud.pointViews[k] = scene.pointcloud.pointViews[idx];
        }
        if (!scene.pointcloud.pointWeights.IsEmpty())
        {
          subScenes[sIdx]->pointcloud.pointWeights[k] = scene.pointcloud.pointWeights[idx];
        }
      }

      // save the final mesh
      const String baseFileName(MAKE_PATH_SAFE(Util::getFullFileName(OPT::strOutputFileName) +
          "_" + std::to_string(i) + "_" + std::to_string(j)));
      subScenes[i*OPT::gridWidth + j]->Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
      subScenes[i*OPT::gridWidth + j]->pointcloud.Save(baseFileName+_T(".ply"));
      subScenes[i*OPT::gridWidth + j]->pointcloud.Release();
    }
  }
  
  Finalize();
  return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
