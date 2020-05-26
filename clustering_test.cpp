// test funtion
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

void clustering(){

    // pointcloud を作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointCloud->points.reserve(/*点の個数*/);

    // 枠の座標をpointcloud の座標変換するところ
    for()}{
        pointCloud->points.emplace_back(pcl::PointXYZ((float)/* x */,(float)/* y */,(float)/* z */));
    }

    pointCloud -> width = pointCloud -> points.size();
    pointCloud -> height = 1;
    pointCloud -> is_dense = true;

    // クラスタリングの設定
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pointCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);
	ec.setInputCloud (pointCloud);

    // クラスタリング実行
    std::vector<pcl::PointIndices> indices;
	ec.extract (indices);

    // クラスタリング の結果を色々できるところ
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){ // グループにアクセスするループ
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // グループ中の点にアクセスするループ
            // 点へのアクセス
            pointCloud -> points[*pit].x;
            pointCloud -> points[*pit].y;
            pointCloud -> points[*pit].z;
        }


    }
}