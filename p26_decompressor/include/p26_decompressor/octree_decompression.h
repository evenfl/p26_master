/*
 * octree_decompression.h
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#ifndef OCTREE_DECOMPRESSION_H_
#define OCTREE_DECOMPRESSION_H_

#include <iterator>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>

#include <ros/ros.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/impl/octree2buf_base.hpp>

#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/impl/octree_pointcloud.hpp>

#include <pcl/compression/entropy_range_coder.h>

using namespace pcl::octree;

namespace wp3 {

typedef pcl::PointXYZI PointT;
typedef OctreePointCloudDensityContainer LeafT;
typedef OctreeContainerEmpty BranchT;
typedef Octree2BufBase<LeafT, LeafT> OctreeT;

class PointCloudDecompression : public OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
{
public:

  typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
  typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
  typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

  // Boost shared pointers
  typedef boost::shared_ptr<PointCloudDecompression > Ptr;
  typedef boost::shared_ptr<const PointCloudDecompression > ConstPtr;

  typedef typename OctreeT::LeafNode LeafNode;
  typedef typename OctreeT::BranchNode BranchNode;


  /** \brief Constructor
   *
   */
  PointCloudDecompression ( bool showStatistics_arg = false) :
    OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (0.04),
    entropy_coder (),
    frame_ID (0),
    point_count (0),
    initialized(false),
    b_show_statistics (showStatistics_arg),
    pointIntensityVector (),
    pointIntensityVectorIterator (){

    frame_header_identifier = "<HEAD>";
    //this->setResolution (octree_resolution_);

  } // End Constructor


  /** \brief Empty deconstructor. */
  virtual ~PointCloudDecompression (){

  }


  /** \brief Provide a pointer to the output data set.
   * \param cloud_arg: the boost shared pointer to a PointCloud message
   */
  inline void setOutputCloud (const PointCloudPtr & cloud_arg)
  {
    if (output != cloud_arg)
    {
      output = cloud_arg;
    }
  }


  /** \brief Decode point cloud from input stream
   *  \param compressed_tree_data_in_arg: binary input stream containing compressed data
   *  \param cloud_arg: reference to decoded point cloud
   */
  void decodePointCloud (std::istream & compressed_tree_data_in_arg, PointCloudPtr & cloud_arg);


private:

  /** \brief Decode leaf nodes information during deserialization
   * \param key_arg octree key of new leaf node
   */
  // param leaf_arg reference to new leaf node
  virtual void deserializeTreeCallback (LeafT & leaf_arg, const OctreeKey & key_arg);


  /** \brief Read frame information to output stream
   *  \param compressed_tree_data_in_arg: binary input stream
   */
  int readFrameHeader (std::istream & compressed_tree_data_in_arg);


  /** \brief Synchronize to frame header
   *  \param compressed_tree_data_in_arg: binary input stream
   */
  void syncToHeader (std::istream & compressed_tree_data_in_arg);


  /** \brief Entropy decoding of input binary stream and output to information vectors
   *  \param compressed_tree_data_in_arg: binary input stream
   */
  void entropyDecoding (std::istream & compressed_tree_data_in_arg);


  /** \brief Pointer to output point cloud dataset. */
  PointCloudPtr output;

  /** \brief Vector for storing binary tree structure */
  std::vector<char> binary_tree_data_vector;

  /** \brief Vector for storing point intensity information  */
  std::vector<char> pointIntensityVector;

  /** \brief Iterator on differential point information vector */
  std::vector<char>::const_iterator pointIntensityVectorIterator;

  /** \brief Static range coder instance */
  pcl::StaticRangeCoder entropy_coder;

  // Settings
  uint32_t frame_ID;
  uint64_t point_count;
  uint64_t compressed_point_data_len;
  bool i_frame;
  bool initialized;

  //bool activating statistics
  bool b_show_statistics;

  //header
  const char * frame_header_identifier;

};

}

#endif /* OCTREE_DECOMPRESSION_H_ */
