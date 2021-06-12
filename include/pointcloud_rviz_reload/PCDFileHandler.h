
#ifndef PCD_FILE_PARSER_H_
#define PCD_FILE_PARSER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <memory.h>

#include <QFile>

namespace pointcloud_rviz_reload
{

class PCDFileHandler
{
public:
   PCDFileHandler() {};
   ~PCDFileHandler() {};

   rviz::PointCloud* convertPCDToPointCloud(const std::string& pcd_path)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr readed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      rviz::PointCloud* target_cloud = new rviz::PointCloud();
      
      // const auto croppedPath = QString::fromStdString(pcd_path).split(".", QString::SkipEmptyParts).at(0);
      // if(QFile::exists(croppedPath + ".bin"))
      //    return convertBINToPointCloud(croppedPath.toStdString() + ".bin");
      // else
      //    convertFromPCDToBIN(croppedPath.toStdString());
      
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_path, *readed_cloud) == -1) //* load the file
      {
         PCL_ERROR ("Couldn't read file : %s\n", pcd_path.c_str());
         return nullptr;
      }
      
      target_cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
      target_cloud->setDimensions(0.05f, 0.05f, 0.05f); // Render object dimension (i.e. spheres)
      target_cloud->setCommonDirection(Ogre::Vector3(0.0, 1.0, 0.0));
      target_cloud->setCommonUpVector(Ogre::Vector3(0.0, 0.0, -1.0));
      target_cloud->setAlpha(1.0);
      std::vector<rviz::PointCloud::Point> points;

      const float scale_factor = 1.0f;
      for (const auto& readed_point: *readed_cloud)
      {
         rviz::PointCloud::Point point;
         point.position.x = readed_point.x * scale_factor;
         point.position.y = readed_point.y * scale_factor;
         point.position.z = readed_point.z * scale_factor;

         point.setColor(readed_point.r, readed_point.g, readed_point.b);
         points.push_back(point);
      }

      target_cloud->addPoints(&points.front(), points.size());

      return target_cloud;
   }
   
   void convertFromPCDToBIN(const std::string& pcd_path)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr readed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_path + ".pcd", *readed_cloud) == -1) //* load the file
      {
         PCL_ERROR ("Couldn't read file : %s\n", pcd_path.c_str());
         return;
      }

      /// \todo. save cloud colors too
		std::ofstream file(pcd_path + ".bin", std::ofstream::binary);
      for(const auto& point : readed_cloud->points)
      {
			file.write((char*)& point.x, sizeof(point.x));
			file.write((char*)& point.y, sizeof(point.y));
			file.write((char*)& point.z, sizeof(point.z));
			// file.write((char*)& point.r, sizeof(point.r));
			// file.write((char*)& point.g, sizeof(point.g));
			// file.write((char*)& point.b, sizeof(point.b));
      }

		std::cout << "success created binarized cloud file: " << std::endl;
		file.close();
   }

   rviz::PointCloud* convertBINToPointCloud(const std::string& bin_path)
   {
		std::ifstream input(bin_path.c_str(), std::ios_base::binary);
		if (!input.good()) 
      {
			std::cerr << "Cannot open file : " << bin_path.c_str() << std::endl;
			return nullptr;
		}

      rviz::PointCloud* target_cloud = new rviz::PointCloud();
      target_cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
      target_cloud->setDimensions(0.05f, 0.05f, 0.05f); // Render object dimension (i.e. spheres)
      target_cloud->setCommonDirection(Ogre::Vector3(0.0, 1.0, 0.0));
      target_cloud->setCommonUpVector(Ogre::Vector3(0.0, 0.0, -1.0));
      target_cloud->setAlpha(1.0);
      std::vector<rviz::PointCloud::Point> points;

		for (int i = 0; input.good() && !input.eof(); i++) 
      {
         rviz::PointCloud::Point point;
			input.read((char *)&point.position.x, 3 * sizeof(typeof(point.position.x)));
			// input.read((char *)&point.color.r, 3 * sizeof(uint8_t));
         points.push_back(point);
		}

		input.close();
      target_cloud->addPoints(&points.front(), points.size());

      return target_cloud;
   }
};
}

#endif