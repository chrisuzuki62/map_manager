/*
    FILE: occupancyMap.cpp
    --------------------------------------
    function definition of occupancy map
*/
#include "map_manager/occupancyMap.h"

namespace mapManager {
    occMap::occMap() : rclcpp::Node("occ_map_node") {
        RCLCPP_INFO(this->get_logger(), "occMap node initialized");
        // this->initMap(std::enable_shared_from_this<occMap>::shared_from_this());
        // RCLCPP_INFO(this->get_logger(), "Initialization complete");
    }
    std::shared_ptr<mapManager::occMap> mapManager::occMap::create() {
        auto instance = std::shared_ptr<occMap>(new occMap());
        instance->initialize();
        return instance;
    }
    void occMap::initialize() {
        // Call initMap after the object is fully constructed
        this->initMap();
    }

	// void occMap::initMap(const std::shared_ptr<rclcpp::Node>& node) {
    void occMap::initMap() {
        // Initialize the parameters, register callbacks, publishers, etc.
        RCLCPP_INFO(this->get_logger(), "Initializing map...");
        initParam();
        registerCallback();
        registerPub();
        RCLCPP_INFO(this->get_logger(), "Publisher initialization complete");
        startVisualization();
        RCLCPP_INFO(this->get_logger(), "Map initialization complete");
    }

    void occMap::initParam() {
        // sensor input mode
        this->declare_parameter(this->ns_ + "sensor_input_mode", 0);
        this->sensorInputMode_ = this->get_parameter(this->ns_ + "sensor_input_mode").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Sensor input mode: depth image (0)/pointcloud (1). Your option: %d", this->hint_.c_str(), this->sensorInputMode_);

        // localization mode
        this->declare_parameter(this->ns_ + "localization_mode", 0);
        this->localizationMode_ = this->get_parameter(this->ns_ + "localization_mode").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Localization mode: pose (0)/odom (1). Your option: %d", this->hint_.c_str(), this->localizationMode_);

        // depth topic name
        this->declare_parameter(this->ns_ + "depth_image_topic", "/camera/depth/image_raw");
        this->depthTopicName_ = this->get_parameter(this->ns_ + "depth_image_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "%s: Depth topic: %s", this->hint_.c_str(), this->depthTopicName_.c_str());

        // pointcloud topic name
        this->declare_parameter(this->ns_ + "point_cloud_topic", "/camera/depth/points");
        this->pointcloudTopicName_ = this->get_parameter(this->ns_ + "point_cloud_topic").as_string();
        RCLCPP_INFO(this->get_logger(), "%s: Pointcloud topic: %s", this->hint_.c_str(), this->pointcloudTopicName_.c_str());

        if (this->localizationMode_ == 0) {
            // pose topic name
            this->declare_parameter(this->ns_ + "pose_topic", "/CERLAB/quadcopter/pose");
            this->poseTopicName_ = this->get_parameter(this->ns_ + "pose_topic").as_string();
            RCLCPP_INFO(this->get_logger(), "%s: Pose topic: %s", this->hint_.c_str(), this->poseTopicName_.c_str());
        }

        if (this->localizationMode_ == 1) {
            // odom topic name
            this->declare_parameter(this->ns_ + "odom_topic", "/CERLAB/quadcopter/odom");
            this->odomTopicName_ = this->get_parameter(this->ns_ + "odom_topic").as_string();
            RCLCPP_INFO(this->get_logger(), "%s: Odom topic: %s", this->hint_.c_str(), this->odomTopicName_.c_str());
        }

        std::vector<double> robotSizeVec = this->declare_parameter<std::vector<double>>(this->ns_ + "robot_size", {0.5, 0.5, 0.3});
        this->robotSize_ = Eigen::Vector3d(robotSizeVec.data());

        std::vector<double> depthIntrinsics = this->declare_parameter<std::vector<double>>(this->ns_ + "depth_intrinsics", {0.0, 0.0, 0.0, 0.0});
        this->fx_ = depthIntrinsics[0];
        this->fy_ = depthIntrinsics[1];
        this->cx_ = depthIntrinsics[2];
        this->cy_ = depthIntrinsics[3];
        RCLCPP_INFO(this->get_logger(), "%s: fx, fy, cx, cy: [%f, %f, %f, %f]", this->hint_.c_str(), this->fx_, this->fy_, this->cx_, this->cy_);

        // depth scale factor
        this->declare_parameter(this->ns_ + "depth_scale_factor", 1000.0);
        this->depthScale_ = this->get_parameter(this->ns_ + "depth_scale_factor").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Depth scale factor: %f", this->hint_.c_str(), this->depthScale_);

        // depth min value
        this->declare_parameter(this->ns_ + "depth_min_value", 0.2);
        this->depthMinValue_ = this->get_parameter(this->ns_ + "depth_min_value").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Depth min value: %f", this->hint_.c_str(), this->depthMinValue_);

        // depth max value
        this->declare_parameter(this->ns_ + "depth_max_value", 5.0);
        this->depthMaxValue_ = this->get_parameter(this->ns_ + "depth_max_value").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Depth max value: %f", this->hint_.c_str(), this->depthMaxValue_);

        // depth filter margin
        this->declare_parameter(this->ns_ + "depth_filter_margin", 0);
        this->depthFilterMargin_ = this->get_parameter(this->ns_ + "depth_filter_margin").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Depth filter margin: %d", this->hint_.c_str(), this->depthFilterMargin_);

        // depth skip pixel
        this->declare_parameter(this->ns_ + "depth_skip_pixel", 1);
        this->skipPixel_ = this->get_parameter(this->ns_ + "depth_skip_pixel").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Depth skip pixel: %d", this->hint_.c_str(), this->skipPixel_);

        // depth image columns
        this->declare_parameter(this->ns_ + "image_cols", 640);
        this->imgCols_ = this->get_parameter(this->ns_ + "image_cols").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Depth image columns: %d", this->hint_.c_str(), this->imgCols_);

        // depth image rows
        this->declare_parameter(this->ns_ + "image_rows", 480);
        this->imgRows_ = this->get_parameter(this->ns_ + "image_rows").as_int();
        RCLCPP_INFO(this->get_logger(), "%s: Depth image rows: %d", this->hint_.c_str(), this->imgRows_);
        this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));

        // transform matrix: body to camera
        std::vector<double> body2CamVec = this->declare_parameter<std::vector<double>>(this->ns_ + "body_to_camera", std::vector<double>(16, 0.0));
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
            }
        }

        // Raycast max length
        this->declare_parameter(this->ns_ + "raycast_max_length", 5.0);
        this->raycastMaxLength_ = this->get_parameter(this->ns_ + "raycast_max_length").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Raycast max length: %f", this->hint_.c_str(), this->raycastMaxLength_);

        // p hit
        double pHit = this->declare_parameter(this->ns_ + "p_hit", 0.70);
        this->pHitLog_ = this->logit(pHit);
        RCLCPP_INFO(this->get_logger(), "%s: P hit: %f", this->hint_.c_str(), pHit);

        // p miss
        double pMiss = this->declare_parameter(this->ns_ + "p_miss", 0.35);
        this->pMissLog_ = this->logit(pMiss);
        RCLCPP_INFO(this->get_logger(), "%s: P miss: %f", this->hint_.c_str(), pMiss);

        // p min
        double pMin = this->declare_parameter(this->ns_ + "p_min", 0.12);
        this->pMinLog_ = this->logit(pMin);
        RCLCPP_INFO(this->get_logger(), "%s: P min: %f", this->hint_.c_str(), pMin);

        // p max
        double pMax = this->declare_parameter(this->ns_ + "p_max", 0.97);
        this->pMaxLog_ = this->logit(pMax);
        RCLCPP_INFO(this->get_logger(), "%s: P max: %f", this->hint_.c_str(), pMax);

        // p occ
        double pOcc = this->declare_parameter(this->ns_ + "p_occ", 0.80);
        this->pOccLog_ = this->logit(pOcc);
        RCLCPP_INFO(this->get_logger(), "%s: P occ: %f", this->hint_.c_str(), pOcc);

        // map resolution
        this->declare_parameter(this->ns_ + "map_resolution", 0.1);
        this->mapRes_ = this->get_parameter(this->ns_ + "map_resolution").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Map resolution: %f", this->hint_.c_str(), this->mapRes_);

        // ground height
        this->declare_parameter(this->ns_ + "ground_height", 0.0);
        this->groundHeight_ = this->get_parameter(this->ns_ + "ground_height").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Ground height: %f", this->hint_.c_str(), this->groundHeight_);

        // map size
        std::vector<double> mapSizeVec(3, 0.0);
        this->declare_parameter<std::vector<double>>("map_size", std::vector<double>{20.0, 20.0, 3.0});
        if (!this->get_parameter("map_size", mapSizeVec)) {
            mapSizeVec = {20.0, 20.0, 3.0};
            RCLCPP_WARN(this->get_logger(), "%s: No map size. Use default: [20, 20, 3].", this->hint_.c_str());
        } else {

            this->mapSize_(0) = mapSizeVec[0];
            this->mapSize_(1) = mapSizeVec[1];
            this->mapSize_(2) = mapSizeVec[2];

            // Initialize min and max sizes
            this->mapSizeMin_(0) = -mapSizeVec[0] / 2; 
            this->mapSizeMax_(0) = mapSizeVec[0] / 2;
            this->mapSizeMin_(1) = -mapSizeVec[1] / 2; 
            this->mapSizeMax_(1) = mapSizeVec[1] / 2;
            this->mapSizeMin_(2) = this->groundHeight_; 
            this->mapSizeMax_(2) = this->groundHeight_ + mapSizeVec[2];
            
            // Min and max for voxels
            this->mapVoxelMin_(0) = 0; 
            this->mapVoxelMax_(0) = std::ceil(mapSizeVec[0] / this->mapRes_);
            this->mapVoxelMin_(1) = 0; 
            this->mapVoxelMax_(1) = std::ceil(mapSizeVec[1] / this->mapRes_);
            this->mapVoxelMin_(2) = 0; 
            this->mapVoxelMax_(2) = std::ceil(mapSizeVec[2] / this->mapRes_);

            // Reserve vector sizes for variables
            int reservedSize = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
            this->countHitMiss_.resize(reservedSize, 0);
            this->countHit_.resize(reservedSize, 0);
            this->occupancy_.resize(reservedSize, this->pMinLog_ - this->UNKNOWN_FLAG_);
            this->occupancyInflated_.resize(reservedSize, false);
            this->flagTraverse_.resize(reservedSize, -1);
            this->flagRayend_.resize(reservedSize, -1);

            RCLCPP_INFO(this->get_logger(), "%s: Map size: [%f, %f, %f]", this->hint_.c_str(), mapSizeVec[0], mapSizeVec[1], mapSizeVec[2]);
        }

        // local update range
        std::vector<double> localUpdateRangeVec = this->declare_parameter<std::vector<double>>(this->ns_ + "local_update_range", {5.0, 5.0, 3.0});
        this->localUpdateRange_ = Eigen::Vector3d(localUpdateRangeVec.data());

        // local bound inflate factor
        this->declare_parameter(this->ns_ + "local_bound_inflation", 0.0);
        this->localBoundInflate_ = this->get_parameter(this->ns_ + "local_bound_inflation").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Local bound inflate: %f", this->hint_.c_str(), this->localBoundInflate_);

        // whether to clean local map
        this->declare_parameter(this->ns_ + "clean_local_map", true);
        this->cleanLocalMap_ = this->get_parameter(this->ns_ + "clean_local_map").as_bool();
        RCLCPP_INFO(this->get_logger(), "%s: Clean local map option is set to: %d", this->hint_.c_str(), this->cleanLocalMap_);

        // absolute dir of prebuilt map file (.pcd)
        this->declare_parameter(this->ns_ + "prebuilt_map_directory", "");
        this->prebuiltMapDir_ = this->get_parameter(this->ns_ + "prebuilt_map_directory").as_string();
        RCLCPP_INFO(this->get_logger(), "%s: The prebuilt map absolute dir is found: %s", this->hint_.c_str(), this->prebuiltMapDir_.c_str());

        // local map size (visualization)
        std::vector<double> localMapSizeVec = this->declare_parameter<std::vector<double>>(this->ns_ + "local_map_size", {10.0, 10.0, 2.0});
        this->localMapSize_ = Eigen::Vector3d(localMapSizeVec.data());

        // max vis height
        this->declare_parameter(this->ns_ + "max_height_visualization", 3.0);
        this->maxVisHeight_ = this->get_parameter(this->ns_ + "max_height_visualization").as_double();
        RCLCPP_INFO(this->get_logger(), "%s: Max visualization height: %f", this->hint_.c_str(), this->maxVisHeight_);

        // visualize global map
        this->declare_parameter(this->ns_ + "visualize_global_map", false);
        this->visGlobalMap_ = this->get_parameter(this->ns_ + "visualize_global_map").as_bool();
        RCLCPP_INFO(this->get_logger(), "%s: Visualize map option. local (0)/global (1): %d", this->hint_.c_str(), this->visGlobalMap_);

        // verbose
        this->declare_parameter(this->ns_ + "verbose", true);
        this->verbose_ = this->get_parameter(this->ns_ + "verbose").as_bool();
        RCLCPP_INFO(this->get_logger(), "%s: Display messages", this->hint_.c_str());
    }

    void occMap::initPrebuiltMap() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(this->prebuiltMapDir_, *cloud) == -1) //* load the file
        {
            RCLCPP_INFO(this->get_logger(), "%s: No prebuilt map found/not using the prebuilt map.", this->hint_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "%s: Map loaded with %lu data points.", this->hint_.c_str(), cloud->width * cloud->height);
            int address;
            Eigen::Vector3i pointIndex;
            Eigen::Vector3d pointPos;
            Eigen::Vector3i inflateIndex;
            int inflateAddress;

            // update occupancy info
            int xInflateSize = ceil(this->robotSize_(0) / (2 * this->mapRes_));
            int yInflateSize = ceil(this->robotSize_(1) / (2 * this->mapRes_));
            int zInflateSize = ceil(this->robotSize_(2) / (2 * this->mapRes_));

            Eigen::Vector3d currMapRangeMin(0.0, 0.0, 0.0);
            Eigen::Vector3d currMapRangeMax(0.0, 0.0, 0.0);

            const int maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
            for (const auto& point : *cloud) {
                address = this->posToAddress(point.x, point.y, point.z);
                pointPos(0) = point.x;
                pointPos(1) = point.y;
                pointPos(2) = point.z;
                this->posToIndex(pointPos, pointIndex);

                this->occupancy_[address] = this->pMaxLog_;
                // update map range
                if (pointPos(0) < currMapRangeMin(0)) {
                    currMapRangeMin(0) = pointPos(0);
                }

                if (pointPos(0) > currMapRangeMax(0)) {
                    currMapRangeMax(0) = pointPos(0);
                }

                if (pointPos(1) < currMapRangeMin(1)) {
                    currMapRangeMin(1) = pointPos(1);
                }

                if (pointPos(1) > currMapRangeMax(1)) {
                    currMapRangeMax(1) = pointPos(1);
                }

                if (pointPos(2) < currMapRangeMin(2)) {
                    currMapRangeMin(2) = pointPos(2);
                }

                if (pointPos(2) > currMapRangeMax(2)) {
                    currMapRangeMax(2) = pointPos(2);
                }

                for (int ix = -xInflateSize; ix <= xInflateSize; ++ix) {
                    for (int iy = -yInflateSize; iy <= yInflateSize; ++iy) {
                        for (int iz = -zInflateSize; iz <= zInflateSize; ++iz) {
                            inflateIndex(0) = pointIndex(0) + ix;
                            inflateIndex(1) = pointIndex(1) + iy;
                            inflateIndex(2) = pointIndex(2) + iz;
                            inflateAddress = this->indexToAddress(inflateIndex);
                            if ((inflateAddress < 0) or (inflateAddress > maxIndex)) {
                                continue; // those points are not in the reserved map
                            }
                            this->occupancyInflated_[inflateAddress] = true;
                        }
                    }
                }
            }
            this->currMapRangeMin_ = currMapRangeMin;
            this->currMapRangeMax_ = currMapRangeMax;
        }
    }

	void occMap::registerCallback() {
		// auto self_shared = std::enable_shared_from_this<mapManager::occMap>::shared_from_this();
		if (this->sensorInputMode_ == 0) {
			// depth pose callback
			// this->depthSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(self_shared, this->depthTopicName_, rmw_qos_profile_default);
            this->depthSub_.subscribe(this, this->depthTopicName_, rmw_qos_profile_default);
			if (this->localizationMode_ == 0) {
				// this->poseSub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(self_shared, this->poseTopicName_, rmw_qos_profile_default);
                this->poseSub_.subscribe(this, this->poseTopicName_, rmw_qos_profile_default);
				this->depthPoseSync_ = std::make_shared<message_filters::Synchronizer<depthPoseSyncPolicy>>(depthPoseSyncPolicy(100), this->depthSub_, this->poseSub_);
				this->depthPoseSync_->registerCallback(std::bind(&occMap::depthPoseCB, this, std::placeholders::_1, std::placeholders::_2));
			} else if (this->localizationMode_ == 1) {
				// this->odomSub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(self_shared, this->odomTopicName_, rmw_qos_profile_default);
				this->odomSub_.subscribe(this, this->odomTopicName_, rmw_qos_profile_default);
                this->depthOdomSync_ = std::make_shared<message_filters::Synchronizer<depthOdomSyncPolicy>>(depthOdomSyncPolicy(100), this->depthSub_, this->odomSub_);
				this->depthOdomSync_->registerCallback(std::bind(&occMap::depthOdomCB, this, std::placeholders::_1, std::placeholders::_2));
			} else {
				RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid localization mode!");
				rclcpp::shutdown();
				exit(0);
			}
		} else if (this->sensorInputMode_ == 1) {
			// pointcloud callback
			// this->pointcloudSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(self_shared, this->pointcloudTopicName_, rmw_qos_profile_default);
			this->pointcloudSub_.subscribe(this, this->pointcloudTopicName_, rmw_qos_profile_default);
            if (this->localizationMode_ == 0) {
				// this->poseSub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(self_shared, this->poseTopicName_, rmw_qos_profile_default);
				this->poseSub_.subscribe(this, this->poseTopicName_, rmw_qos_profile_default);
                this->pointcloudPoseSync_ = std::make_shared<message_filters::Synchronizer<pointcloudPoseSyncPolicy>>(pointcloudPoseSyncPolicy(100), this->pointcloudSub_, this->poseSub_);
				this->pointcloudPoseSync_->registerCallback(std::bind(&occMap::pointcloudPoseCB, this, std::placeholders::_1, std::placeholders::_2));
			} else if (this->localizationMode_ == 1) {
				// this->odomSub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(self_shared, this->odomTopicName_, rmw_qos_profile_default);
				this->odomSub_.subscribe(this, this->odomTopicName_, rmw_qos_profile_default);
                this->pointcloudOdomSync_ = std::make_shared<message_filters::Synchronizer<pointcloudOdomSyncPolicy>>(pointcloudOdomSyncPolicy(100), this->pointcloudSub_, this->odomSub_);
				this->pointcloudOdomSync_->registerCallback(std::bind(&occMap::pointcloudOdomCB, this, std::placeholders::_1, std::placeholders::_2));
			} else {
				RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid localization mode!");
				rclcpp::shutdown();
				exit(0);
			}
		} else {
			RCLCPP_ERROR(this->get_logger(), "[OccMap]: Invalid sensor input mode!");
			rclcpp::shutdown();
			exit(0);
		}
        this->occTimer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&occMap::updateOccupancyCB, this));

		// map inflation callback
		this->inflateTimer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&occMap::inflateMapCB, this));

        this->visTimer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&occMap::visCB, this));
		this->visWorker_ = std::thread(&occMap::startVisualization, this);
		// this->visWorker_.detach();
	}





    void occMap::registerPub() {
        this->depthCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_ + "/depth_cloud", 10);
        this->mapVisPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_ + "/voxel_map", 10);
        this->inflatedMapVisPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_ + "/inflated_voxel_map", 10);
        this->map2DPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(this->ns_ + "/two_D_occupancy_map", 10);
        this->mapExploredPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->ns_ + "/explored_voxel_map", 10);
        // publish service
        this->collisionCheckServer_ = this->create_service<map_manager::srv::CheckPosCollision>(this->ns_ + "/check_pos_collision", std::bind(&occMap::checkCollision, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool occMap::checkCollision(const std::shared_ptr<map_manager::srv::CheckPosCollision::Request> req, std::shared_ptr<map_manager::srv::CheckPosCollision::Response> res) {
        if (req->inflated) {
            res->occupied = this->isInflatedOccupied(Eigen::Vector3d(req->x, req->y, req->z));
        } else {
            res->occupied = this->isOccupied(Eigen::Vector3d(req->x, req->y, req->z));
        }

        return true;
    }

    void occMap::depthPoseCB(const std::shared_ptr<const sensor_msgs::msg::Image> img, const std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose) {
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix;
        this->getCameraPose(pose, camPoseMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

        if (this->isInMap(this->position_)) {
            this->occNeedUpdate_ = true;
        } else {
            this->occNeedUpdate_ = false;
        }
    }

    void occMap::depthOdomCB(const std::shared_ptr<const sensor_msgs::msg::Image> img, const std::shared_ptr<const nav_msgs::msg::Odometry> odom) {
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix;
        this->getCameraPose(odom, camPoseMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

        if (this->isInMap(this->position_)) {
            this->occNeedUpdate_ = true;
        } else {
            this->occNeedUpdate_ = false;
        }
    }

    void occMap::pointcloudPoseCB(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud, const std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose) {
        // directly get the point cloud
        pcl::PCLPointCloud2 pclPC2;
        pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
        pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix;
        this->getCameraPose(pose, camPoseMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

        if (this->isInMap(this->position_)) {
            this->occNeedUpdate_ = true;
        } else {
            this->occNeedUpdate_ = false;
        }
    }

    void occMap::pointcloudOdomCB(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud, const std::shared_ptr<const nav_msgs::msg::Odometry> odom) {
        // directly get the point cloud
        cout<<"reading pc"<<endl;
        pcl::PCLPointCloud2 pclPC2;
        pcl_conversions::toPCL(*pointcloud, pclPC2); // convert ros pointcloud2 to pcl pointcloud2
        pcl::fromPCLPointCloud2(pclPC2, this->pointcloud_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix;
        this->getCameraPose(odom, camPoseMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);
        // cout<< "In Map: " << this->isInMap(this->position_) << endl;
        // cout<< "position x: " << this->position_(0)<< "position y: "<< this->position_(1)<< "position z: "<< this->position_(2) << endl;
        if (this->isInMap(this->position_)) {
            this->occNeedUpdate_ = true;
        } else {
            this->occNeedUpdate_ = false;
        }
    }

void occMap::updateOccupancyCB() {
        if (not this->occNeedUpdate_){
			return;
		}
		cout << "update occupancy map" << endl;
		// ros::Time startTime, endTime;
		
		// startTime = ros::Time::now();
		if (this->sensorInputMode_ == 0){
			// project 3D points from depth map
			this->projectDepthImage();
		}
		else if (this->sensorInputMode_ == 1){
			// directly get pointcloud
			this->getPointcloud();
		}

		// raycasting and update occupancy
		this->raycastUpdate();


		// clear local map
		if (this->cleanLocalMap_){
			this->cleanLocalMap();
		}
		
		// inflate map
		// this->inflateLocalMap();
		// endTime = ros::Time::now();
		if (this->verbose_){
			// cout << this->hint_ << ": Occupancy update time: " << (endTime - startTime).toSec() << " s." << endl;
		}
		this->occNeedUpdate_ = false;
		this->mapNeedInflate_ = true;
	}


    void occMap::inflateMapCB() {
        // inflate local map:
		if (this->mapNeedInflate_){
			this->inflateLocalMap();
			this->mapNeedInflate_ = false;
			this->esdfNeedUpdate_ = true;
		}
	}

    void occMap::projectDepthImage() {
        this->projPointsNum_ = 0;

		int cols = this->depthImage_.cols;
		int rows = this->depthImage_.rows;
		uint16_t* rowPtr;

		Eigen::Vector3d currPointCam, currPointMap;
		double depth;
		const double inv_factor = 1.0 / this->depthScale_;
		const double inv_fx = 1.0 / this->fx_;
		const double inv_fy = 1.0 / this->fy_;


		// iterate through each pixel in the depth image
		for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
			rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
			for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
				depth = (*rowPtr) * inv_factor;
				
				// if (*rowPtr == 0) {
				// 	depth = this->raycastMaxLength_ + 0.1;
				// 	rowPtr =  rowPtr + this->skipPixel_;
				// 	continue;
				// } else if (depth < this->depthMinValue_) {
				// 	continue;
				// } else if (depth > this->depthMaxValue_ and depth < 1.5 * this->depthMaxValue_) {
				// 	depth = this->raycastMaxLength_ + 0.1;
				// }
				// else if (depth >= 1.5 * this->depthMaxValue_){
				// 	rowPtr =  rowPtr + this->skipPixel_;
				// 	continue;
				// }

				if (*rowPtr == 0) {
					depth = this->raycastMaxLength_ + 0.1;
				} else if (depth < this->depthMinValue_) {
					continue;
				} else if (depth > this->depthMaxValue_ ) {
					depth = this->raycastMaxLength_ + 0.1;
				}

				rowPtr =  rowPtr + this->skipPixel_;

				// get 3D point in camera frame
				currPointCam(0) = (u - this->cx_) * depth * inv_fx;
				currPointCam(1) = (v - this->cy_) * depth * inv_fy;
				currPointCam(2) = depth;
				currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

				if (this->useFreeRegions_){ // this region will not be updated and directly set to free
					if (this->isInHistFreeRegions(currPointMap)){
						continue;
					}
				}

				// store current point
				this->projPoints_[this->projPointsNum_] = currPointMap;
				this->projPointsNum_ = this->projPointsNum_ + 1;
			}
		} 
	}

    void occMap::getPointcloud() {
        this->projPointsNum_ = this->pointcloud_.size();
		this->projPoints_.resize(this->projPointsNum_);
		Eigen::Vector3d currPointCam, currPointMap;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPointCam(0) = this->pointcloud_.points[i].x;
			currPointCam(1) = this->pointcloud_.points[i].y;
			currPointCam(2) = this->pointcloud_.points[i].z;
			currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate
			if ((currPointMap-this->position_).norm()>=0.5){
				this->projPoints_[i] = currPointMap;
			}
		}
	}

    void occMap::raycastUpdate() {
        if (this->projPointsNum_ == 0){
			return;
		}
		this->raycastNum_ += 1;

		// record local bound of update
		double xmin, xmax, ymin, ymax, zmin, zmax;
		xmin = xmax = this->position_(0);
		ymin = ymax = this->position_(1);
		zmin = zmax = this->position_(2);

		// iterate through each projected points, perform raycasting and update occupancy
		Eigen::Vector3d currPoint;
		bool pointAdjusted;
		int rayendVoxelID, raycastVoxelID;
		double length;
		for (int i=0; i<this->projPointsNum_; ++i){
			currPoint = this->projPoints_[i];
			if (std::isnan(currPoint(0)) or std::isnan(currPoint(1)) or std::isnan(currPoint(2))){
				continue; // nan points can happen when we are using pointcloud as input
			}

			pointAdjusted = false;
			// check whether the point is in reserved map range
			if (not this->isInMap(currPoint)){
				currPoint = this->adjustPointInMap(currPoint);
				pointAdjusted = true;
			}

			// check whether the point exceeds the maximum raycasting length
			length = (currPoint - this->position_).norm();
			if (length > this->raycastMaxLength_){
				currPoint = this->adjustPointRayLength(currPoint);
				pointAdjusted = true;
			}


			// update local bound
			if (currPoint(0) < xmin){xmin = currPoint(0);}
			if (currPoint(1) < ymin){ymin = currPoint(1);}
			if (currPoint(2) < zmin){zmin = currPoint(2);}
			if (currPoint(0) > xmax){xmax = currPoint(0);}
			if (currPoint(1) > ymax){ymax = currPoint(1);}
			if (currPoint(2) > zmax){zmax = currPoint(2);}

			// update occupancy itself update information
			rayendVoxelID = this->updateOccupancyInfo(currPoint, not pointAdjusted); // point adjusted is free, not is occupied

			// check whether the voxel has already been updated, so no raycasting needed
			// rayendVoxelID = this->posToAddress(currPoint);
			if (this->flagRayend_[rayendVoxelID] == this->raycastNum_){
				continue; // skip
			}
			else{
				this->flagRayend_[rayendVoxelID] = this->raycastNum_;
			}



			// raycasting for update occupancy
			this->raycaster_.setInput(currPoint/this->mapRes_, this->position_/this->mapRes_);
			Eigen::Vector3d rayPoint, actualPoint;
			while (this->raycaster_.step(rayPoint)){
				actualPoint = rayPoint;
				actualPoint(0) += 0.5;
				actualPoint(1) += 0.5;
				actualPoint(2) += 0.5;
				actualPoint *= this->mapRes_;
				raycastVoxelID = this->updateOccupancyInfo(actualPoint, false);

				// raycastVoxelID = this->posToAddress(actualPoint);
				if (this->flagTraverse_[raycastVoxelID] == this->raycastNum_){
					break;
				}
				else{
					this->flagTraverse_[raycastVoxelID] = this->raycastNum_;
				}

			}
		}

		// store local bound and inflate local bound (inflate is for ESDF update)
		this->posToIndex(Eigen::Vector3d (xmin, ymin, zmin), this->localBoundMin_);
		this->posToIndex(Eigen::Vector3d (xmax, ymax, zmax), this->localBoundMax_);
		this->localBoundMin_ -= int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); // inflate in x y direction
		this->localBoundMax_ += int(ceil(this->localBoundInflate_/this->mapRes_)) * Eigen::Vector3i(1, 1, 0); 
		this->boundIndex(this->localBoundMin_); // since inflated, need to bound if not in reserved range
		this->boundIndex(this->localBoundMax_);


		// update occupancy in the cache
		double logUpdateValue;
		int cacheAddress, hit, miss;
		while (not this->updateVoxelCache_.empty()){
			Eigen::Vector3i cacheIdx = this->updateVoxelCache_.front();
			this->updateVoxelCache_.pop();
			cacheAddress = this->indexToAddress(cacheIdx);

			hit = this->countHit_[cacheAddress];
			miss = this->countHitMiss_[cacheAddress] - hit;

			if (hit >= miss and hit != 0){
				logUpdateValue = this->pHitLog_;
			}
			else{
				logUpdateValue = this->pMissLog_;
			}
			this->countHit_[cacheAddress] = 0; // clear hit
			this->countHitMiss_[cacheAddress] = 0; // clear hit and miss

			// check whether point is in the local update range
			if (not this->isInLocalUpdateRange(cacheIdx)){
				continue; // do not update if not in the range
			}

			if (this->useFreeRegions_){ // current used in simulation, this region will not be updated and directly set to free
				Eigen::Vector3d pos;
				this->indexToPos(cacheIdx, pos);
				if (this->isInHistFreeRegions(pos)){
					this->occupancy_[cacheAddress] = this->pMinLog_;
					continue;
				}
			}

			// update occupancy info
			if ((logUpdateValue >= 0) and (this->occupancy_[cacheAddress] >= this->pMaxLog_)){
				continue; // not increase p if max clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] == this->pMinLog_)){
				continue; // not decrease p if min clamped
			}
			else if ((logUpdateValue <= 0) and (this->occupancy_[cacheAddress] < this->pMinLog_)){
				this->occupancy_[cacheAddress] = this->pMinLog_; // if unknown set it free (prior), 
				continue;
			}

			this->occupancy_[cacheAddress] = std::min(std::max(this->occupancy_[cacheAddress]+logUpdateValue, this->pMinLog_), this->pMaxLog_);

			// update the entire map range (if it is not unknown)
			if (not this->isUnknown(cacheIdx)){
				Eigen::Vector3d cachePos;
				this->indexToPos(cacheIdx, cachePos);
				if (cachePos(0) > this->currMapRangeMax_(0)){
					this->currMapRangeMax_(0) = cachePos(0);
				}
				else if (cachePos(0) < this->currMapRangeMin_(0)){
					this->currMapRangeMin_(0) = cachePos(0);
				}

				if (cachePos(1) > this->currMapRangeMax_(1)){
					this->currMapRangeMax_(1) = cachePos(1);
				}
				else if (cachePos(1) < this->currMapRangeMin_(1)){
					this->currMapRangeMin_(1) = cachePos(1);
				}

				if (cachePos(2) > this->currMapRangeMax_(2)){
					this->currMapRangeMax_(2) = cachePos(2);
				}
				else if (cachePos(2) < this->currMapRangeMin_(2)){
					this->currMapRangeMin_(2) = cachePos(2);
				}
			}
		}

	}

    void occMap::cleanLocalMap() {
        Eigen::Vector3i posIndex;
		this->posToIndex(this->position_, posIndex);
		Eigen::Vector3i innerMinBBX = posIndex - this->localMapVoxel_;
		Eigen::Vector3i innerMaxBBX = posIndex + this->localMapVoxel_;
		Eigen::Vector3i outerMinBBX = innerMinBBX - Eigen::Vector3i(5, 5, 5);
		Eigen::Vector3i outerMaxBBX = innerMaxBBX + Eigen::Vector3i(5, 5, 5);
		this->boundIndex(innerMinBBX);
		this->boundIndex(innerMaxBBX);
		this->boundIndex(outerMinBBX);
		this->boundIndex(outerMaxBBX);

		// clear x axis
		for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int x=outerMinBBX(0); x<=innerMinBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int x=innerMaxBBX(0); x<=outerMaxBBX(0); ++x){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;					
				}
			}
		}
        // clear y axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int z=outerMinBBX(2); z<=outerMaxBBX(2); ++z){
				for (int y=outerMinBBX(1); y<=innerMinBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int y=innerMaxBBX(1); y<=outerMaxBBX(1); ++y){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}

		// clear z axis
		for (int x=outerMinBBX(0); x<=outerMaxBBX(0); ++x){
			for (int y=outerMinBBX(1); y<=outerMaxBBX(1); ++y){
				for (int z=outerMinBBX(2); z<=innerMinBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}

				for (int z=innerMaxBBX(2); z<=outerMaxBBX(2); ++z){
					this->occupancy_[this->indexToAddress(Eigen::Vector3i (x, y, z))] = this->pMinLog_ - this->UNKNOWN_FLAG_;
				}
			}
		}
	}

    void occMap::inflateLocalMap() {
		int xmin = this->localBoundMin_(0);
		int xmax = this->localBoundMax_(0);
		int ymin = this->localBoundMin_(1);
		int ymax = this->localBoundMax_(1);
		int zmin = this->localBoundMin_(2);
		int zmax = this->localBoundMax_(2);
		Eigen::Vector3i clearIndex;
		// clear previous data in current data range
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					clearIndex(0) = x; clearIndex(1) = y; clearIndex(2) = z;
					this->occupancyInflated_[this->indexToAddress(clearIndex)] = false;
				}
			}
		}

		int xInflateSize = ceil(this->robotSize_(0)/(2*this->mapRes_));
		int yInflateSize = ceil(this->robotSize_(1)/(2*this->mapRes_));
		int zInflateSize = ceil(this->robotSize_(2)/(2*this->mapRes_));

		// inflate based on current occupancy
		Eigen::Vector3i pointIndex, inflateIndex;
		int inflateAddress;
		const int  maxIndex = this->mapVoxelMax_(0) * this->mapVoxelMax_(1) * this->mapVoxelMax_(2);
		for (int x=xmin; x<=xmax; ++x){
			for (int y=ymin; y<=ymax; ++y){
				for (int z=zmin; z<=zmax; ++z){
					pointIndex(0) = x; pointIndex(1) = y; pointIndex(2) = z;
					if (this->isOccupied(pointIndex)){
						for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
							for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
								for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
									inflateIndex(0) = pointIndex(0) + ix;
									inflateIndex(1) = pointIndex(1) + iy;
									inflateIndex(2) = pointIndex(2) + iz;
									inflateAddress = this->indexToAddress(inflateIndex);
									if ((inflateAddress < 0) or (inflateAddress > maxIndex)){
										continue; // those points are not in the reserved map
									} 
									this->occupancyInflated_[inflateAddress] = true;
								}
							}
						}
					}
				}
			}
		}
	}


    void occMap::visCB() {
        // cout<< "Vis CB" <<endl;
        this->publishProjPoints();
        // this->publishMap();
        this->publishInflatedMap();
        this->publish2DOccupancyGrid();
    }

    void occMap::startVisualization() {
        // rclcpp::Rate rate(10);
        // while (rclcpp::ok()) {
        //     this->publishProjPoints();
        //     this->publishMap();
        //     rate.sleep();
        // }
    }

    void occMap::getMapVisData(pcl::PointCloud<pcl::PointXYZ>& mapCloud, pcl::PointCloud<pcl::PointXYZ>& inflatedMapCloud, pcl::PointCloud<pcl::PointXYZ>& exploredMapCloud, pcl::PointCloud<pcl::PointXYZ>& depthCloud) {
        pcl::PointXYZ pt;
		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			depthCloud.push_back(pt);
		}

		depthCloud.width = depthCloud.points.size();
		depthCloud.height = 1;
		depthCloud.is_dense = true;
		depthCloud.header.frame_id = "map";

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							mapCloud.push_back(pt);
						}
					}
					
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							inflatedMapCloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredMapCloud.push_back(pt);
					}
				}
			}
		}

		mapCloud.width = mapCloud.points.size();
		mapCloud.height = 1;
		mapCloud.is_dense = true;
		mapCloud.header.frame_id = "map";

		inflatedMapCloud.width = inflatedMapCloud.points.size();
		inflatedMapCloud.height = 1;
		inflatedMapCloud.is_dense = true;
		inflatedMapCloud.header.frame_id = "map";

		exploredMapCloud.width = exploredMapCloud.points.size();
		exploredMapCloud.height = 1;
		exploredMapCloud.is_dense = true;
		exploredMapCloud.header.frame_id = "map";
	}


    void occMap::publishProjPoints() {
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		for (int i=0; i<this->projPointsNum_; ++i){
			pt.x = this->projPoints_[i](0);
			pt.y = this->projPoints_[i](1);
			pt.z = this->projPoints_[i](2);
			cloud.push_back(pt);
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::msg::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->depthCloudPub_->publish(cloudMsg);
        cout<< "Published points" <<endl;
	}


    void occMap::publishMap() {
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> exploredCloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);

					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}

					// publish explored voxel map
					if(!this->isUnknown(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						pt.x = point(0);
						pt.y = point(1);
						pt.z = point(2);
						exploredCloud.push_back(pt);
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		exploredCloud.width = exploredCloud.points.size();
		exploredCloud.height = 1;
		exploredCloud.is_dense = true;
		exploredCloud.header.frame_id = "map";

		sensor_msgs::msg::PointCloud2 cloudMsg;
		sensor_msgs::msg::PointCloud2 exploredCloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		pcl::toROSMsg(exploredCloud, exploredCloudMsg);
		this->mapVisPub_->publish(cloudMsg);
		this->mapExploredPub_->publish(exploredCloudMsg);

        cout<< "Published map" <<endl;
	}


    void occMap::publishInflatedMap() {
		pcl::PointXYZ pt;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		Eigen::Vector3d minRange, maxRange;
		if (this->visGlobalMap_){
			// minRange = this->mapSizeMin_;
			// maxRange = this->mapSizeMax_;
			minRange = this->currMapRangeMin_;
			maxRange = this->currMapRangeMax_;
		}
		else{
			minRange = this->position_ - localMapSize_;
			maxRange = this->position_ + localMapSize_;
			minRange(2) = this->groundHeight_;
		}
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				for (int z=minRangeIdx(2); z<=maxRangeIdx(2); ++z){
					Eigen::Vector3i pointIdx (x, y, z);
					// if (this->occupancy_[this->indexToAddress(pointIdx)] > this->pMinLog_){
					if (this->isInflatedOccupied(pointIdx)){
						Eigen::Vector3d point;
						this->indexToPos(pointIdx, point);
						if (point(2) <= this->maxVisHeight_){
							pt.x = point(0);
							pt.y = point(1);
							pt.z = point(2);
							cloud.push_back(pt);
						}
					}
				}
			}
		}

		cloud.width = cloud.points.size();
		cloud.height = 1;
		cloud.is_dense = true;
		cloud.header.frame_id = "map";

		sensor_msgs::msg::PointCloud2 cloudMsg;
		pcl::toROSMsg(cloud, cloudMsg);
		this->inflatedMapVisPub_->publish(cloudMsg);	
        cout<< "Published inflated map" <<endl;
	}


    void occMap::publish2DOccupancyGrid() {
       Eigen::Vector3d minRange, maxRange;
		minRange = this->mapSizeMin_;
		maxRange = this->mapSizeMax_;
		minRange(2) = this->groundHeight_;
		Eigen::Vector3i minRangeIdx, maxRangeIdx;
		this->posToIndex(minRange, minRangeIdx);
		this->posToIndex(maxRange, maxRangeIdx);
		this->boundIndex(minRangeIdx);
		this->boundIndex(maxRangeIdx);

		nav_msgs::msg::OccupancyGrid mapMsg;
		for (int i=0; i<maxRangeIdx(0); ++i){
			for (int j=0; j<maxRangeIdx(1); ++j){
				mapMsg.data.push_back(0);
			}
		}

		double z = 0.5;
		int zIdx = int(z/this->mapRes_);
		for (int x=minRangeIdx(0); x<=maxRangeIdx(0); ++x){
			for (int y=minRangeIdx(1); y<=maxRangeIdx(1); ++y){
				Eigen::Vector3i pointIdx (x, y, zIdx);
				int map2DIdx = x  +  y * maxRangeIdx(0);
				if (this->isUnknown(pointIdx)){
					mapMsg.data[map2DIdx] = -1;
				}
				else if (this->isOccupied(pointIdx)){
					mapMsg.data[map2DIdx] = 100;
				}
				else{
					mapMsg.data[map2DIdx] = 0;
				}
			}
		}
		mapMsg.header.frame_id = "map";
		mapMsg.info.resolution = this->mapRes_;
		mapMsg.info.width = maxRangeIdx(0);
		mapMsg.info.height = maxRangeIdx(1);
		mapMsg.info.origin.position.x = minRange(0);
		mapMsg.info.origin.position.y = minRange(1);
		this->map2DPub_->publish(mapMsg);
        cout<< "Published 2d map" <<endl;		
	}

} // namespace mapManager

