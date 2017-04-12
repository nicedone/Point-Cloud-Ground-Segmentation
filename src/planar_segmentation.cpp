#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>

#include <iostream>


void planeSeg(pcl::SACSegmentation<pcl::PointXYZRGB> segmentation,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr
planeIndices)
{
  // Get the plane model, if present.
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01f);
  segmentation.setOptimizeCoefficients(true);
  segmentation.segment(*planeIndices, *coefficients);
  printf("num_points: %d\n",planeIndices->indices.size());
}

int main(int argc, char** argv)
{
  /* Objects for storing point clouds at different stages */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

  /* Read in PCD file */
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
  {
    return -1;
  }

  /* Perform plane segmentation */
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  // stores plane function
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  planeSeg(segmentation,cloud,coefficients,planeIndices);
  
  // Print plane coefficients: ax+by+cz+d=0
  /*for (int i=0;i<4;i++)
  {
    printf("coef[%d]:%f,",i,coefficients->values[i]);
  }*/
  //printf("\n");

  pcl::visualization::CloudViewer viewerObjects("Objects on table");
  if (planeIndices->indices.size() == 0)
    std::cout << "Could not find a plane in the scene." << std::endl;
  else
  {
    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    //extract.setNegative(true);
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*plane);
    /*viewerObjects.showCloud(plane);
    while (!viewerObjects.wasStopped())
    {
        // Do nothing but wait.
    }*/
  
    // Retrieve the convex hull.
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(plane);
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(2);
    hull.reconstruct(*convexHull);

    // Redundant check.*/
    if (hull.getDimension() == 2)
    {
      // Prism object.
      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
      // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
      prism.setHeightLimits(0.0f, 0.3f);
      pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

      prism.segment(*objectIndices);

      // Get and show all points retrieved by the hull.
      //pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setIndices(objectIndices);
      extract.filter(*objects);
      
    }
    else std::cout << "The chosen hull is not planar." << std::endl;
  }    
    
  // Perform second planar segmentation
  // TODO: WHY DO I NEED TO DO PLANE SEGMENTATION AGAIN
  planeSeg(segmentation,objects,coefficients,planeIndices);
   
  /* Remove Floor Plane from Point Cloud */
  pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
  extract2.setNegative(true);
  extract2.setInputCloud(objects);
  extract2.setIndices(planeIndices);
  extract2.filter(*plane);

  //pcl::visualization::CloudViewer viewerObjects("Objects on table");
  /*viewerObjects.showCloud(plane);
  while (!viewerObjects.wasStopped())
  {
    // Do nothing but wait.
  }*/
  
  /* Two Options for Projecting onto Floor Plane:
     1: Transform to (0,0,0) orientation, then set Z coordinate to 0
     2. Project to Floor Plane in Place */
  /* Option 1 */

  /* Option 2 */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (plane);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  
  viewerObjects.showCloud(cloud_projected);
  while (!viewerObjects.wasStopped())
  {
    // Do nothing but wait.
  }
}
