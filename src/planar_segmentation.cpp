#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>

#include <string>
#include <iostream>

#define PLANEERROR 0.01
#define OBJECTHEIGHTMAX 0.3
#define OBJECTHEIGHTMIN 0.0


/* 
   Wrapper Function for savePCDFileBinary
   Saves Point Cloud into .pcd format
 */
void savePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
             std::string& path,
             const std::string& newExt){

  std::string::size_type i = path.rfind('.', path.length());
  // Boolean used for determining if point cloud is of object or of
  // object's projection.
  
  path.replace(i, newExt.length(), newExt);
  pcl::io::savePCDFileBinary (path, *pointCloud);
  printf("saving point cloud to %s\n",path);

}
       
/* 
   Wrapper Function for CloudViewer
   Used to view point cloud while code is running.
 */
void viewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  pcl::visualization::CloudViewer pclViewer("Objects on table");
  pclViewer.showCloud(pointCloud);
  while (!pclViewer.wasStopped())
  {
      // Do nothing but wait.
  }
}

/* 
  Finds plane models in point cloud.
  coefficients: stores plane function
  planeIndices: stores points that lie on the newly found plane
 */
void planeSeg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
              pcl::ModelCoefficients::Ptr coefficients,
              pcl::PointIndices::Ptr planeIndices)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(PLANEERROR);
  segmentation.setOptimizeCoefficients(true);
  segmentation.segment(*planeIndices, *coefficients);
}

/*
  Projects object points onto tabletop plane defined in coefficients
 */
void projectPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_cloud,
                   pcl::ModelCoefficients::Ptr coefficients,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (plane);
  proj.setModelCoefficients (coefficients);
  proj.filter (*proj_cloud);
}


/*
  Removes either the points on the plane from the entire point cloud
  or only keeps the points on the plane. This distinction is defined
  by setNeg (true deletes points on the plane).
 */
void removePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                 pcl::PointIndices::Ptr cloudIndices,
                 bool setNeg)
{
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setNegative(setNeg);
  extract.setInputCloud(inputCloud);
  extract.setIndices(cloudIndices);
  extract.filter(*outCloud);
}


int main(int argc, char** argv)
{
  /* Objects for storing point clouds at different stages */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);

  /* Read in PCD file */
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *init_cloud) != 0)
  {
    return -1;
  }

  /* Perform plane segmentation */
  pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  planeSeg(init_cloud,plane_coef,planeIndices);

  if (planeIndices->indices.size() == 0)
  {
    std::cout << "Could not find a plane in the scene." << std::endl;
    return -1;
  }
  else
  {
    // Copy the points of the plane to a new cloud.
    removePlane(plane,init_cloud,planeIndices,false);

    // Retrieve the convex hull.
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(plane);
    
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(2);
    hull.reconstruct(*convexHull);

    // Prism object.
    pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
    prism.setInputCloud(init_cloud);
    prism.setInputPlanarHull(convexHull);

    // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
    // Second parameter: maximum Z value. Tune it to the expected height of the object.
    prism.setHeightLimits(OBJECTHEIGHTMIN, OBJECTHEIGHTMAX);
    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

    prism.segment(*objectIndices);

    // Get points retrieved by the hull.
    removePlane(objects,init_cloud,objectIndices,false);
  }
    
  // Perform second planar segmentation
  planeSeg(objects,plane_coef,planeIndices);
  
  /* Remove Floor Plane from Point Cloud */
  removePlane(plane, objects, planeIndices, true);

  /* Save point cloud of object */
  std::string path = argv[1];
  savePCD(plane,path,"_obj.pcd");

  /* uncomment to view point cloud while code is running */
  //viewPointCloud(plane);

  /* Project Object Points to Tabletop plane 2 */
  projectPoints(cloud_projected, plane_coef, plane);
  
  /* Save point cloud of projection */
  path = argv[1];
  savePCD(cloud_projected,path,"_proj.pcd");
  
  /* uncomment to view point cloud while code is running */
  //viewPointCloud(cloud_projected);
  
}
