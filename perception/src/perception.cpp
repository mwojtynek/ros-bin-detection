// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/StdVector>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

//Includes aus dem Beispiel: http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Includes aus dem Beispiel: Cluster&Bounding http://www.pcl-users.org/clustering-and-a-bounding-box-td3905570.html
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>

//Includes aus dem Beispiel: Cluster http://answers.ros.org/question/52935/flann-pcl-segmentation-fault-with-kdtree-in-euclidean-cluster-extraction/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <stdlib.h> // rand()
#include <time.h>   // time
#include <math.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/Point.h>
#include <pcl/common/transforms.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Messages
#include <geometry_msgs/Point.h>

// Transform
#include <tf/transform_listener.h>

// MoveIt!
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group/move_group_context.h>
#include <std_srvs/Empty.h>

#define tableDistance 1.349
#define boxHeight 147

class perception {

public:
    perception();

    visualization_msgs::Marker setMarker(std::string ns ,int id, float r, float g, float b, int action);
    visualization_msgs::Marker mark_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string axis, double from, double to);
    void getObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, int clusterId);
    void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input);
    void setTransform();
    void clearOctomap();

protected:
    ros::NodeHandle nh;
    ros::Publisher pub_sensor;
    ros::Publisher pub_marker;
    ros::Publisher pub_octoMap;
    ros::Publisher pub_boxCoord;
    ros::Publisher pub_boxAngle;
    ros::Subscriber sub;
    tf::TransformBroadcaster br1;
    // opencv calculation variables
    float boxAngle;
    cv::Point2f boxCenter;
    pcl::PointXYZ cloudMinPoint, cloudMaxPoint;
    float depth;
    Eigen::Vector3f dim;
    float blob_angle_deg;
    double objectLength;
    double objectWidth;
    // marker
    visualization_msgs::Marker marker;
    double boxMarkerHeight;
    // transform
    geometry_msgs::PointStamped boxPoint;
    geometry_msgs::PointStamped boxOrientation;
    geometry_msgs::PointStamped boxTransOut;
    tf::TransformListener listener;
};

// Constructor
perception::perception(){

    // Create a ROS publisher for the output point cloud
    pub_sensor = nh.advertise<sensor_msgs::PointCloud2> ("perception/pcl_output", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker> ("perception/box_marker", 1);
    pub_octoMap = nh.advertise<sensor_msgs::PointCloud2> ("perception/octoMap", 1);
    pub_boxCoord = nh.advertise<geometry_msgs::PointStamped> ("perception/box_center", 1);
    pub_boxAngle = nh.advertise<geometry_msgs::PointStamped> ("perception/box_angle", 1);
    sub = nh.subscribe ("camera_top/camera/depth_registered/points", 1, &perception::cloud_cb, this);
}

void perception::clearOctomap(){
    // Clearing and Refreshing OctoMap in Rviz
    ros::NodeHandle nh;
    ros::ServiceClient clear_octomap_service_client_= nh.serviceClient<std_srvs::Empty>("clear_octomap");
    std_srvs::Empty srv;
    clear_octomap_service_client_.call(srv);
}

void perception::getObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, int clusterId)
{
    // Conversion from PointXYZRGB to PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_cluster,*cloud);

    Eigen::Matrix4f projectOntoXYPlane (Eigen::Matrix4f::Identity());
    projectOntoXYPlane.col(2) = Eigen::Vector4f(0,0,0,0);

    // Datatypes
    pcl::PointCloud<pcl::PointXYZ>::Ptr flatPointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);

    // project points onto xy-plane
    pcl::transformPointCloud(*cloud, *flatPointcloud, projectOntoXYPlane);

    // compute centroid of original point cloud
    Eigen::Vector4f cloudCentroid ;
    pcl::compute3DCentroid(*cloud, cloudCentroid);

    // Create a Convex Hull representation of the projected point cloud
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (flatPointcloud);
    chull.reconstruct (*cloud_hull);

    // get min max point of original point cloud
    pcl::getMinMax3D(*cloud, cloudMinPoint, cloudMaxPoint);

    // convert convenx hull points pcl::PointXYZ into cv::Point2f and move them into the std::vector
    std::vector<cv::Point2f> points;
    for(unsigned int ii = 0; ii < cloud_hull->points.size(); ii++)
    {
        cv::Point2f p2d(cloud_hull->points[ii].x, cloud_hull->points[ii].y );
        points.push_back(p2d);
    }

    // convert vector to matrix
    cv::Mat points_matrix(points);

    // get minimum oriented bounding rectangle
    cv::RotatedRect orientetBoundingRectangle = cv::minAreaRect(points_matrix);

    // get demensions of bounding rectangle
    boxCenter = orientetBoundingRectangle.center;
    cv::Size2f boxSize = orientetBoundingRectangle.size;
    boxAngle = orientetBoundingRectangle.angle * M_PI / 180; //convert rotation angle to radians

    // calculate depth of original point cloud
    depth = cloudMaxPoint.z - cloudMinPoint.z;

    // get distance of object/box
    Eigen::Vector3f distanceBox;
    distanceBox << cloudMinPoint.getVector3fMap();
    boxMarkerHeight = (tableDistance - distanceBox[2]);

    // bounding box dimenisons
    dim << boxSize.width, boxSize.height, (boxMarkerHeight);

    // getting box angle in range of 0 to +90 and -90 to 0
    cv::RotatedRect rotated_rect = cv::minAreaRect(points_matrix);
    blob_angle_deg = rotated_rect.angle;
    if (rotated_rect.size.width < rotated_rect.size.height) {
      blob_angle_deg = 90 + blob_angle_deg;
    }

    // initialisation
    objectLength = -1;
    objectWidth = -1;

    if (boxSize.width > boxSize.height)
    {
        objectLength = boxSize.width;
        objectWidth = boxSize.height;
    }
    else if (boxSize.width < boxSize.height)
    {
        objectLength = boxSize.height;
        objectWidth = boxSize.width;
    }

    // box size
    struct box{
        double width;
        double length;
        double height;
        double noisedelta;
    };
    // Bigbox (B x L x H) 19.5 x 30 x 14.7 -- Smallbox (B x L x H) 14.5 x 19.5 x 14.7
    struct box bigbox = {0.195, 0.30, 0.147, 0.06};
    struct box smallbox = {0.145, 0.195, 0.147, 0.06};

    // bigbox and smallbox calculations related to noisedelta
    double bbsumlength = bigbox.length + bigbox.noisedelta;
    double bbsumwidth = bigbox.width + bigbox.noisedelta;
    double bbsumheight = bigbox.height + bigbox.noisedelta;
    double bbdifflength = bigbox.length - bigbox.noisedelta;
    double bbdiffwidth = bigbox.width - bigbox.noisedelta;
    double bbdiffheight = bigbox.height - bigbox.noisedelta;

    double sbsumlength = smallbox.length + smallbox.noisedelta;
    double sbsumwidth = smallbox.width + smallbox.noisedelta;
    double sbsumheight = smallbox.height + smallbox.noisedelta;
    double sbdifflength = smallbox.length - smallbox.noisedelta;
    double sbdiffwidth = smallbox.width - smallbox.noisedelta;
    double sbdiffheight = smallbox.height - smallbox.noisedelta;

    // Bigbox
    if ( ((bbdiffwidth <= objectWidth) && ( objectWidth <= bbsumwidth))
      &&  ((bbdifflength <= objectLength) && (objectLength <= bbsumlength))
      &&  ((bbdiffheight <= boxMarkerHeight) && (boxMarkerHeight <= bbsumheight))
         )
    {
        std::cout << "Big Box gefunden id =" << clusterId<<std::endl;
        pub_marker.publish(setMarker("bigbox", clusterId,1.0,0.0,0.0,0));
        setTransform();
    }
    // Smallbox
    else if ( ((sbdiffwidth <= objectWidth) && ( objectWidth <= sbsumwidth))
          &&  ((sbdifflength <= objectLength) && (objectLength <= sbsumlength))
          &&  ((sbdiffheight <= boxMarkerHeight) && (boxMarkerHeight <= sbsumheight)) )
    {
        std::cout << "Small Box gefunden id =" << clusterId <<std::endl;
        pub_marker.publish(setMarker("smallbox", clusterId,1.0,0.0,0.0,0));
        setTransform();
    }
    // Obstacle
    else
    {
        std::cout << "keine Box gefunden cluser id = " << clusterId <<std::endl;
        // Publish the dataSize in sensor_msgs type
        pub_marker.publish(setMarker("obstacle", clusterId,1.0,0.0,0.0,2));
        // conversion
//        pcl::PCLPointCloud2 cloud_obstacle_output;
//        pcl::toPCLPointCloud2(*cloud_cluster, cloud_obstacle_output);
        sensor_msgs::PointCloud2 cloud_obstacle_output;
//        pcl::toROSMsg(*cloud_cluster, cloud_obstacle_output);
//        clearOctomap();
        pub_octoMap.publish(cloud_obstacle_output);
      }

}

visualization_msgs::Marker perception::setMarker(std::string ns ,int id, float r, float g, float b, int action)
{
    // translation vector
    Eigen::Vector3f translation;
    translation << boxCenter.x, boxCenter.y, (cloudMinPoint.z + boxMarkerHeight/2);

    // conversion from camera frame to base_link and providing quaternion for box orientation
    geometry_msgs::QuaternionStamped box_quat;
    box_quat.quaternion = tf::createQuaternionMsgFromYaw(boxAngle);

    // ############ Marker ############
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "camera_rgb_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    if (action == 0) {
        marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
       marker.action = visualization_msgs::Marker::DELETE;
    }
    marker.pose.position.x = translation(0);
    marker.pose.position.y = translation(1);
    marker.pose.position.z = translation(2);
    marker.pose.orientation = box_quat.quaternion;
    // marker scale
    marker.scale.x = dim(0);
    marker.scale.y = dim(1);
    marker.scale.z = dim(2);
    if (marker.scale.x ==0)
        marker.scale.x=0.1;

    if (marker.scale.y ==0)
      marker.scale.y=0.1;

    if (marker.scale.z ==0)
      marker.scale.z=0.1;
    // marker color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(0.5);

return marker;
}

void perception::setTransform()
{
    // preparing box coordinates for transformation
    boxPoint.point.x = boxCenter.x;
    boxPoint.point.y = boxCenter.y;
    boxPoint.point.z = cloudMaxPoint.z;
    boxPoint.header.frame_id = "camera_rgb_optical_frame";          // camera source frame

    // preparing box angle for sending topic
    boxOrientation.point.x = 0;
    boxOrientation.point.y = 0;
    boxOrientation.point.z = (blob_angle_deg * M_PI / 180);
    boxOrientation.header.frame_id = "camera_rgb_optical_frame";

    // conversion boxPoint from -camera frame- to -base_link-
    std::string output_frame="base_link";
     try{
        listener.waitForTransform(output_frame, boxPoint.header.frame_id, ros::Time(0), ros::Duration(10.0));
        listener.transformPoint(output_frame, boxPoint, boxTransOut);
     }
     catch( tf::TransformException ex)
     {
         ROS_ERROR("transfrompose exception : %s",ex.what());
     }

    // Publishing Topic
    pub_boxCoord.publish(boxTransOut);
    pub_boxAngle.publish(boxOrientation);
}



visualization_msgs::Marker perception::mark_center(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b)
{
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;

    pcl::compute3DCentroid (*cloud_cluster, centroid);
    pcl::getMinMax3D (*cloud_cluster, min, max);

    visualization_msgs::Marker point;
    // point definition
    point.header.frame_id = "camera_rgb_optical_frame";
    point.header.stamp = ros::Time::now();
    point.ns=ns;
    point.id = id;
    point.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    point.scale.x = ((max[0]-min[0])/10);
    point.scale.y = ((max[1]-min[1])/10);

    // Points are green
    point.color.g = 1.0f;
    point.color.a = 1.0;


    //pcl::tracking::ParticleFilterTracker<PointInT,StateT>::calcBoundingBox 	();

    // coordinates of point

    geometry_msgs::Point p;
    p.x = centroid[0];
    p.y = centroid[1];
    p.z = centroid[2];

    point.points.push_back(p);

    point.lifetime = ros::Duration();
    return point;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr perception::passthroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string axis, double from, double to)
{
    // passthrough filter for adjusting/cutting the field of view
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(from, to);
    pass.filter(*cloudFiltered);

    return cloudFiltered;
}

void perception::cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
    // Datatypes
    pcl::PCLPointCloud2 cloud_output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Other Operators
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

    // PCL Conversion to ROS
    pcl::fromPCLPointCloud2(*input, *cloud_raw);
    // XYZ-Passthrough
    //std::cout << "Punkte vorher: " << cloud_raw->points.size() << std::endl;
    cloud_raw = passthroughFilter(cloud_raw, "x", -0.55, 0.425);     // cut from - to in meters
    cloud_raw = passthroughFilter(cloud_raw, "y", -0.5, 0.4);
    cloud_raw = passthroughFilter(cloud_raw, "z", 0.1, tableDistance);
    //std::cout << "Punkte nachher: " << cloud_raw->points.size() << std::endl;
    // Voxelgrid downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_raw);
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (*cloud_filtered);
    // ############ Outputs Remove Surface - Tischplatte entfernen ############

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    //extract.setNegative (false);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter(*cloud_without_plane);     // Output
    //extract_normals.setNegative (true);
    //extract_normals.setInputCloud (cloud_normals);
    //extract_normals.setIndices (inliers_plane);
    //extract_normals.filter (*cloud_without_plane);
    // ############ statistical outlier removal ############
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_new;
    sor_new.setInputCloud (cloud_without_plane);
    sor_new.setMeanK (50);
    sor_new.setStddevMulThresh (1.0);
    sor_new.filter (*cloud_without_plane);    // Output
    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);

    // ############ Clustering ############
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 ( new pcl::search::KdTree<pcl::PointXYZRGB>() );
    tree->setInputCloud(cloud_without_plane);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance( 0.1 );
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod( tree2 );
    ec.setInputCloud(cloud_without_plane);
    ec.extract(cluster_indices);
    srand (time(NULL));

    // Process each cluster
    int i =0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {
        // random color
        i++;
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;

        // 	temporary pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        // extract all points of ONE cluster
        for (std::vector<int>::const_iterator pit =
                     it->indices.begin(); pit != it->indices.end(); pit++)
        {
            pcl::PointXYZRGB point = cloud_without_plane->points[*pit];
            point.r = r;
            point.g = g;
            point.b = b;
            cloud_cluster->points.push_back(point);
            tmp_cloud_cluster->points.push_back(point);

        }

        tmp_cloud_cluster->width = tmp_cloud_cluster->points.size();
        tmp_cloud_cluster->height = 1;
        tmp_cloud_cluster->sensor_origin_ = cloud_without_plane->sensor_origin_;
        tmp_cloud_cluster->sensor_orientation_ = cloud_without_plane->sensor_orientation_;
        tmp_cloud_cluster->header = cloud_without_plane->header;

        // function calls
//        clearOctomap();
        perception::getObject(tmp_cloud_cluster,i);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->sensor_origin_ = cloud_without_plane->sensor_origin_;
    cloud_cluster->sensor_orientation_ = cloud_without_plane->sensor_orientation_;
    cloud_cluster->header = cloud_without_plane->header;

    // Publish the dataSize in sensor_msgs type
    pcl::toPCLPointCloud2(*cloud_cluster, cloud_output);
    pub_sensor.publish (cloud_output);

//    sub.shutdown();
} // end void


int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "perception");
    // Class Variable
    perception prc;
    ros::spin();
    ros::shutdown();
    return(0);
  }
