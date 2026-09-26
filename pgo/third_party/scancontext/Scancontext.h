#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI;
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


void coreImportTest ( void );

// sc param-independent helper functions
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public:
    SCManager( ) = default;

    Eigen::MatrixXd makeScancontext( const pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 );
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 );
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 );

    // User-side API
    void makeAndSaveScancontextAndKeys( const pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void );
    std::pair<int, float> detectLoopClosureIDGivenScan( const pcl::PointCloud<SCPointType> & _scan_down );

    // for ltslam
    void saveScancontextAndKeys( Eigen::MatrixXd _scd );
    std::pair<int, float> detectLoopClosureIDBetweenSession ( std::vector<float>& curr_key,  Eigen::MatrixXd& curr_desc);

    const Eigen::MatrixXd& getConstRefRecentSCD(void);

    // Rebuild KD-tree from loaded data (call after deserializing polarcontext_invkeys_mat_)
    void rebuildTree();

    // Match a given scan against the full database (no NUM_EXCLUDE_RECENT)
    std::pair<int, float> detectBestMatchForScan( const pcl::PointCloud<SCPointType> & _scan_down );

public:
    // hyper parameters
    const double LIDAR_HEIGHT = 2.0;

    const int    PC_NUM_RING = 20;
    const int    PC_NUM_SECTOR = 60;
    double PC_MAX_RADIUS = 80.0;
    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    const double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    const int    NUM_EXCLUDE_RECENT = 30;
    const int    NUM_CANDIDATES_FROM_TREE = 3;

    // loop thres
    const double SEARCH_RATIO = 0.1;
    double SC_DIST_THRES = 0.2;

    // config
    const int    TREE_MAKING_PERIOD_ = 30;
    int          tree_making_period_conter = 0;

    // setter
    void setSCdistThres(double _new_thres);
    void setMaximumRadius(double _max_r);

    // data
    std::vector<double> polarcontexts_timestamp_;
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    bool is_tree_batch_made = false;
    std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;

}; // SCManager
