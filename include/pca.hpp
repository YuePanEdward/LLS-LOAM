
#ifndef _INCLUDE_COMMON_PCA_H_
#define _INCLUDE_COMMON_PCA_H_

//pcl
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
//opencv2
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>

//our
#include <types.h>

#include <vector>

namespace map_pose
{
struct eigenValue // Eigen Value ,lamada1 > lamada2 > lamada3;
{
	double lamada1;
	double lamada2;
	double lamada3;
};

struct eigenVector //the eigen vector corresponding to the eigen value
{
	Eigen::Vector3f principalDirection;
	Eigen::Vector3f middleDirection;
	Eigen::Vector3f normalDirection;
};

struct pcaFeature //PCA
{
	eigenValue values;
	eigenVector vectors;
	double curvature;
	double linear;
	double planar;
	double spherical;
	double linear_2;
	double planar_2;
	double spherical_2;
	pcl::PointNormal pt;
	size_t ptId;
	size_t ptNum = 0;
	std::vector<int> neighbor_indices;
};

template <typename Point_T>
class PrincipleComponentAnalysis
{
public:
	/**
		* \brief Estimate the normals of the input Point Cloud by PCL speeding up with OpenMP
		* \param[in] inputPointCloud is the input Point Cloud Pointer
		* \param[in] radius is the neighborhood search radius (m) for KD Tree
		* \param[out] normals is the normal of all the points from the Point Cloud
		*/
	bool CalculateNormalVector_Radius(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
									  float radius,
									  pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		// Create the normal estimation class, and pass the input dataset to it;
		pcl::NormalEstimationOMP<Point_T, pcl::Normal> ne;
		ne.setNumberOfThreads(1); //More threads sometimes would not speed up the procedure
		ne.setInputCloud(inputPointCloud);
		// Create an empty kd-tree representation, and pass it to the normal estimation object;
		typename pcl::search::KdTree<Point_T>::Ptr tree(new pcl::search::KdTree<Point_T>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius;
		ne.setRadiusSearch(radius);
		// Compute the normal
		ne.compute(*normals);
		CheckNormals(normals);
		return true;
	}

	bool CalculatePointCloudWithNormal_Radius(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
											  float radius,
											  pcl::PointCloud<pcl::PointNormal>::Ptr &pointnormals)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		bool normal_ready = CalculateNormalVector_Radius(inputPointCloud, radius, normals);
		if (normal_ready)
		{
			// Concatenate the XYZ and normal fields*
			pcl::concatenateFields(*inputPointCloud, *normals, *pointnormals);
			return true;
		}
		else
			return false;
	}

	bool CalculateNormalVector_KNN(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
								   int K,
								   pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		// Create the normal estimation class, and pass the input dataset to it;
		pcl::NormalEstimationOMP<Point_T, pcl::Normal> ne;
		ne.setNumberOfThreads(1);
		ne.setInputCloud(inputPointCloud);
		// Create an empty kd-tree representation, and pass it to the normal estimation object;
		typename pcl::search::KdTree<Point_T>::Ptr tree(new pcl::search::KdTree<Point_T>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius;
		ne.setKSearch(K);
		// Compute the normal
		ne.compute(*normals);
		CheckNormals(normals);
		return true;
	}

	bool CalculateNormalVector_KNN(std::shared_ptr<point_cloud_t<PointType>> &cloud_in,
								   std::vector<unsigned int> &ground_idx,
								   int K)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		for (int i = 0; i < ground_idx.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x= cloud_in->points[ground_idx[i]].x;
			pt.y= cloud_in->points[ground_idx[i]].y;
			pt.z= cloud_in->points[ground_idx[i]].z;
			cloud_pcl->points.push_back(pt);
		}

		// Create the normal estimation class, and pass the input dataset to it;
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNumberOfThreads(2);
		ne.setInputCloud(cloud_pcl);
		// Create an empty kd-tree representation, and pass it to the normal estimation object;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius;
		ne.setKSearch(K);
		// Compute the normal
		ne.compute(*normals);
		CheckNormals(normals);

		for (int i = 0; i < ground_idx.size(); i++)
		{
			cloud_in->points[ground_idx[i]].nx = normals->points[i].normal_x;
			cloud_in->points[ground_idx[i]].ny = normals->points[i].normal_y;
			cloud_in->points[ground_idx[i]].nz = normals->points[i].normal_z;
		}

		return true;
	}

	bool CalculatePointCloudWithNormal_KNN(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
										   int K,
										   pcl::PointCloud<pcl::PointNormal>::Ptr &pointnormals)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		bool normal_ready = CalculateNormalVector_KNN(inputPointCloud, K, normals);
		if (normal_ready)
		{
			// Concatenate the XYZ and normal fields*
			pcl::concatenateFields(*inputPointCloud, *normals, *pointnormals);
			return true;
		}
		else
			return false;
	}

	/**
		* \brief Principle Component Analysis (PCA) of the Point Cloud with fixed search radius
		* \param[in] inputPointCloud is the input Point Cloud (XYZI) Pointer
		* \param[in]     radius is the neighborhood search radius (m) for KD Tree
		* \param[out]features is the pcaFeature vector of all the points from the Point Cloud
		*/
	bool CalculatePcaFeaturesOfPointCloud(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
										  std::vector<pcaFeature> &features,
										  const float radius)
	{
		//LOG(INFO) << "input cloud size is " << inputPointCloud->points.size();
		pcl::KdTreeFLANN<Point_T> tree;
		tree.setInputCloud(inputPointCloud);
		features.resize(inputPointCloud->size());

		//concurrency::parallel_for(size_t(0), inputPointCloud->points.size(), [&](size_t i)   //Multi-thread
		//{
		for (int i = 0; i < inputPointCloud->points.size(); i++)
		{
			std::vector<int> search_indices; //point index Vector
			std::vector<float> distances;	//distance Vector
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			tree.radiusSearch(i, radius, search_indices, distances); //KD tree
			features[i].pt = inputPointCloud->points[i];
			features[i].ptId = i;
			features[i].ptNum = search_indices.size();
			CalculatePcaFeature(inputPointCloud, search_indices, features[i]);
			// inputPointCloud->points[i].normal_x =  features[i].vectors.normalDirection.x();
			// inputPointCloud->points[i].normal_y =  features[i].vectors.normalDirection.y();
			// inputPointCloud->points[i].normal_z =  features[i].vectors.normalDirection.z();
		}
		//});

		return true;
	}

	bool CalculatePcaFeaturesOfPointCloud(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
										  std::vector<pcaFeature> &features,
										  const int K)
	{
		//LOG(INFO) << "input cloud size is " << inputPointCloud->points.size();
		pcl::KdTreeFLANN<Point_T> tree;
		tree.setInputCloud(inputPointCloud);
		features.resize(inputPointCloud->size());

		//concurrency::parallel_for(size_t(0), inputPointCloud->points.size(), [&](size_t i)   //Multi-thread
		//{
		for (int i = 0; i < inputPointCloud->points.size(); i++)
		{
			std::vector<int> search_indices; //point index Vector
			std::vector<float> distances;	//distance Vector
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			tree.nearestKSearch(i, K, search_indices, distances); //KD tree
			features[i].pt = inputPointCloud->points[i];
			features[i].ptId = i;
			features[i].ptNum = search_indices.size();
			CalculatePcaFeature(inputPointCloud, search_indices, features[i]);
			// inputPointCloud->points[i].normal_x =  features[i].vectors.normalDirection.x();
			// inputPointCloud->points[i].normal_y =  features[i].vectors.normalDirection.y();
			// inputPointCloud->points[i].normal_z =  features[i].vectors.normalDirection.z();
		}
		//});

		return true;
	}

	/**
		* \brief Use OpenCV to accomplish the Principle Component Analysis (PCA)
		* of one point and its neighborhood
		* \param[in] inputPointCloud is the input Point Cloud Pointer
		* \param[in] search_indices is the neighborhood points' indices of the search point.
		* \param[out]feature is the pcaFeature of the search point.
		*/
	bool CalculatePcaFeature(typename pcl::PointCloud<Point_T>::Ptr inputPointCloud,
							 std::vector<int> &search_indices,
							 pcaFeature &feature)
	{
		size_t ptNum;
		ptNum = search_indices.size();

		if (ptNum < 3)
			return false;

		CvMat *pData = cvCreateMat(ptNum, 3, CV_32FC1);
		CvMat *pMean = cvCreateMat(1, 3, CV_32FC1);
		CvMat *pEigVals = cvCreateMat(1, 3, CV_32FC1);
		CvMat *pEigVecs = cvCreateMat(3, 3, CV_32FC1);

		for (size_t i = 0; i < ptNum; ++i)
		{
			cvmSet(pData, i, 0, inputPointCloud->points[search_indices[i]].x);
			cvmSet(pData, i, 1, inputPointCloud->points[search_indices[i]].y);
			cvmSet(pData, i, 2, inputPointCloud->points[search_indices[i]].z);
		}

		cvCalcPCA(pData, pMean, pEigVals, pEigVecs, CV_PCA_DATA_AS_ROW);

		feature.vectors.principalDirection.x() = cvmGet(pEigVecs, 0, 0);
		feature.vectors.principalDirection.y() = cvmGet(pEigVecs, 0, 1);
		feature.vectors.principalDirection.z() = cvmGet(pEigVecs, 0, 2);

		// feature.vectors.middleDirection.x() = cvmGet(pEigVecs, 1, 0);
		// feature.vectors.middleDirection.y() = cvmGet(pEigVecs, 1, 1);
		// feature.vectors.middleDirection.z() = cvmGet(pEigVecs, 1, 2);

		feature.vectors.normalDirection.x() = cvmGet(pEigVecs, 2, 0);
		feature.vectors.normalDirection.y() = cvmGet(pEigVecs, 2, 1);
		feature.vectors.normalDirection.z() = cvmGet(pEigVecs, 2, 2);

		feature.values.lamada1 = cvmGet(pEigVals, 0, 0);
		feature.values.lamada2 = cvmGet(pEigVals, 0, 1);
		feature.values.lamada3 = cvmGet(pEigVals, 0, 2);

		if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
		{
			feature.curvature = 0;
		}
		else
		{
			feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);
		}

		// feature.linear = (sqrt(feature.values.lamada1) - sqrt(feature.values.lamada2)) / sqrt(feature.values.lamada1);
		// feature.planar = (sqrt(feature.values.lamada2) - sqrt(feature.values.lamada3)) / sqrt(feature.values.lamada1);
		// feature.spherical = sqrt(feature.values.lamada3) / sqrt(feature.values.lamada1);
		feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) / (feature.values.lamada1);
		feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) / (feature.values.lamada1);
		feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);
		cvReleaseMat(&pEigVecs);
		cvReleaseMat(&pEigVals);
		cvReleaseMat(&pMean);
		cvReleaseMat(&pData);

		search_indices.swap(feature.neighbor_indices);
		return true;
	}

protected:
private:
	/**
		* \brief Check the Normals (if they are finite)
		* \param  normals is the input Point Cloud (XYZI)'s Normal Pointer
		*/
	void CheckNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		//It is advisable to check the normals before the call to compute()
		for (int i = 0; i < normals->points.size(); i++)
		{
			if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
			{
				normals->points[i].normal_x = 0.577; // 1/ sqrt(3)
				normals->points[i].normal_y = 0.577;
				normals->points[i].normal_z = 0.577;
				normals->points[i].curvature = 0.0;
			}

			// if (_isnan(normals->points[i].curvature))
			// {
			// 	normals->points[i].normal_x = 0.577;
			// 	normals->points[i].normal_y = 0.577;
			// 	normals->points[i].normal_z = 0.577;
			// 	normals->points[i].curvature = 0.0;
			// }
		}
	}
};

} // namespace map_pose
#endif //_INCLUDE_COMMON_PCA_H_
