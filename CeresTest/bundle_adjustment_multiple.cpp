#include "bundle_adjustment_multiple.h"
#define LEN_IMAGE_NAME 5
#define NUM_POINTS 88

#define K3OFF
//#define RANDOMIZE
class BundleAdjustmentProblem {
public:
	~BundleAdjustmentProblem() {
		delete[] chb_detected_;
		delete[] image_point_measurements_;
		delete[] img_names_;
		delete[] cam_params;
	}
	double* camera_parameters(int cam_idx) {
		// return first index of camera parameters array for the camera index
		return cam_params + cam_idx * 15;
	}
	double* world_point_estimates(int frame_idx, int point_idx) {
		// return 88 world points for a frame
		return world_points + (frame_idx * NUM_POINTS * 3) + point_idx * 3;
	}


	double image_point_measurements(int frame_idx, int cam_idx, int img_idx, int dim) {
		return image_point_measurements_[frame_idx * NUM_CAMERA * NUM_POINTS * 2 + cam_idx * NUM_POINTS * 2 + img_idx * 2 + dim];
	}
	const bool chb_detected(int frame_idx, int cam_idx) const {
		return chb_detected_[frame_idx * NUM_CAMERA + cam_idx];
	}
	int num_frames() const {
		return num_frames_;
	}
	bool LoadCameraParameters(const char* xml_path) {
#ifdef RANDOMIZE
		srand(time(NULL));
#endif
		FILE* camparam_fptr = fopen(xml_path, "r");
		if (camparam_fptr == NULL) {
			std::cerr << "ERROR: unable to open file: " << xml_path << "\n";
			std::getchar();
			return false;
		}

		cam_params = new double[15 * NUM_CAMERA];
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			double ax1, ax2, ax3, t1, t2, t3, fx, fy, c1, c2, k1, k2, p1, p2, k3;
			int cam_idx_confirm;
			fscanf(camparam_fptr, "%d", &cam_idx_confirm);

#ifdef RANDOMIZE
			ax1 = (rand() % 100 + 1)*0.05;
			ax2 = (rand() % 100 + 1)*0.05;
			ax3 = (rand() % 100 + 1)*0.05;

			t1 = (rand() % 3000 + 1);
			t2 = (rand() % 3000 + 1);
			t3 = (rand() % 3000 + 1);

			fx = (rand() % 2000 + 1000);
			fy = (rand() % 2000 + 1000);

			c1 = (rand() % 2000 + 1000);
			c2 = (rand() % 2000 + 1000);

			k1 = (rand() % 100 + 1)*0.0001;
			k2 = (rand() % 100 + 1)*0.0001;
			p1 = (rand() % 100 + 1)*0.0001;
			p2 = (rand() % 100 + 1)*0.0001;
			k3 = (rand() % 100 + 1)*0.0001;
#else
			fscanf(camparam_fptr, "%lf", &ax1);
			fscanf(camparam_fptr, "%lf", &ax2);
			fscanf(camparam_fptr, "%lf", &ax3);

			fscanf(camparam_fptr, "%lf", &t1);
			fscanf(camparam_fptr, "%lf", &t2);
			fscanf(camparam_fptr, "%lf", &t3);

			fscanf(camparam_fptr, "%lf", &fx);
			fscanf(camparam_fptr, "%lf", &fy);

			fscanf(camparam_fptr, "%lf", &c1);
			fscanf(camparam_fptr, "%lf", &c2);

			fscanf(camparam_fptr, "%lf", &k1);
			fscanf(camparam_fptr, "%lf", &k2);
			fscanf(camparam_fptr, "%lf", &p1);
			fscanf(camparam_fptr, "%lf", &p2);
			fscanf(camparam_fptr, "%lf", &k3);
#endif
			cam_params[cam_idx * 15 + 0] = ax1;
			cam_params[cam_idx * 15 + 1] = ax2;
			cam_params[cam_idx * 15 + 2] = ax3;

			cam_params[cam_idx * 15 + 3] = t1;
			cam_params[cam_idx * 15 + 4] = t2;
			cam_params[cam_idx * 15 + 5] = t3;

			cam_params[cam_idx * 15 + 6] = fx;
			cam_params[cam_idx * 15 + 7] = fy;

			cam_params[cam_idx * 15 + 8] = c1;
			cam_params[cam_idx * 15 + 9] = c2;

			cam_params[cam_idx * 15 + 10] = k1;
			cam_params[cam_idx * 15 + 11] = k2;
			cam_params[cam_idx * 15 + 12] = p1;
			cam_params[cam_idx * 15 + 13] = p2;
			cam_params[cam_idx * 15 + 14] = k3;
		}
		cout << "Done parsing: " << xml_path << endl;
		return true;
	}
	bool LoadDataset(const char* filename) {
		/* initialize random seed: */
		srand(time(NULL));


		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			return false;
		}

		FscanfOrDie(fptr, "%d", &num_frames_);

		chb_detected_ = new bool[NUM_CAMERA * num_frames_];
		image_point_measurements_ = new double[NUM_CAMERA * NUM_POINTS * 2 * num_frames_](); // initialize to 0s
		img_names_ = (char **)malloc((LEN_IMAGE_NAME + 1) * num_frames_ * sizeof(char*));
		world_points = new double[num_frames_* NUM_POINTS * 3];

		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			// for each frame
			FscanfOrDie(fptr, "%s", &img_name_[0]);
			img_names_[frame_idx] = (char*)malloc((LEN_IMAGE_NAME + 1) * sizeof(char));
			strcpy(img_names_[frame_idx], img_name_);

			FscanfOrDie(fptr, "%d", &num_cams_saw_chb);

			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				FscanfOrDie(fptr, "%d", &curr_camera_idx_);

				int chb_detected;
				FscanfOrDie(fptr, "%d", &chb_detected);
				chb_detected_[frame_idx * NUM_CAMERA + cam_idx] = (chb_detected == 1);

				if (chb_detected == 1) {
					for (int imgpoint_idx = 0; imgpoint_idx < NUM_POINTS; imgpoint_idx++) {
						double u, v;
						FscanfOrDie(fptr, "%lf", &u);
						FscanfOrDie(fptr, "%lf", &v);
						image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + imgpoint_idx) * 2 + 0] = u;
						image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + imgpoint_idx) * 2 + 1] = v;
					}
				}
				else {
				}
			}
		}

		// 3d world points estimates (88 points per image)
		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			FscanfOrDie(fptr, "%s", &img_name_[0]);
			for (int p_idx = 0; p_idx < NUM_POINTS; p_idx++) {
				double X, Y, Z;
#ifdef RANDOMIZE
				X = (rand() % 6000 + 1)*-0.5;
				Y = (rand() % 3000 + 1);
				Z = (rand() % 2000 + 1);
#else
				FscanfOrDie(fptr, "%lf", &X);
				FscanfOrDie(fptr, "%lf", &Y);
				FscanfOrDie(fptr, "%lf", &Z);
#endif
				world_points[(frame_idx*NUM_POINTS * 3) + (p_idx * 3) + 0] = X;
				world_points[(frame_idx*NUM_POINTS * 3) + (p_idx * 3) + 1] = Y;
				world_points[(frame_idx*NUM_POINTS * 3) + (p_idx * 3) + 2] = Z;
			}
		}
#if 0
		// check data input
		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			if (frame_idx < 160 || frame_idx > 170) continue;
			printf("frame[%d] image_name: %s ===============================\n", frame_idx, img_names_[frame_idx]);
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				bool dtd = chb_detected_[frame_idx * NUM_CAMERA + cam_idx];
				if (dtd > 0) {
					double u1 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 0];
					double v1 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 1];

					double u2 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 87) * 2 + 0];
					double v2 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 87) * 2 + 1];
					printf("cam[%d]: (%.4lf, %.4lf), (%.4lf, %.4lf)\n", cam_idx, u1, v1, u2, v2);
				}
				else {
					printf("cam[%d]:\n", cam_idx);
				}
			}
			printf("\n");
		}
		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			cout << frame_idx << "| " << img_names_[frame_idx] << ": " << world_points[frame_idx*NUM_POINTS * 3 + 0] << ", " << world_points[frame_idx*NUM_POINTS * 3 + 1] << ", " << world_points[frame_idx*NUM_POINTS * 3 + 2] << endl;
		}
#endif
		return true;
	}
	char** img_names_;
	
	// world point estimates
	double *world_points; // num_frames * 88 * 3
	double *cam_params; // 15*8
private:
	template<typename T>
	void FscanfOrDie(FILE *fptr, const char* format, T *value) {
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1) {
			LOG(FATAL) << "invalid UW data file.";
		}
	}
	char img_name_[LEN_IMAGE_NAME + 1];
	int num_cams_saw_chb;
	int num_frames_;
	bool detected_;
	bool *chb_detected_;
	int curr_camera_idx_;

	double* image_point_measurements_; // 2 x 8 x num_frames
};
struct ReprojectionError {
	double observed_x, observed_y; // size 8
	ReprojectionError(double observed_xi, double observed_yi) : observed_x(observed_xi), observed_y(observed_yi) {
	}

	template<typename T>
	bool operator()(const T* const camera, const T* const world_point, T* residuals) const {
		const T ang_axis[3] = { camera[0], camera[1], camera[2] };
		const T trans[3] = { camera[3], camera[4], camera[5] };
		const T f[2] = { camera[6], camera[7] };
		const T c[2] = { camera[8], camera[9] };
		const T k[3] = { camera[10], camera[11], camera[14] }; // k3 is excluded here. camera[14]
		const T p[2] = { camera[12], camera[13] };

		T cam_point[3];
		// angle-axis rotation
		ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

		// translation
		cam_point[0] += trans[0];
		cam_point[1] += trans[1];
		cam_point[2] += trans[2];

		// center of distortion
		T xp = cam_point[0] / cam_point[2];
		T yp = cam_point[1] / cam_point[2];

		// distortions
		T r2 = xp * xp + yp * yp;
#ifdef K3OFF
		T radial_dist = 1.0 + r2 * (k[0] + k[1] * r2);
#else
		T radial_dist = 1.0 + r2 * (k[0] + k[1] * r2 + k[2] * r2*r2);
#endif
		T tangential_dist_x = T(2.0) * p[0] * xp*yp + p[1] * (r2 + T(2.0) * xp*xp);
		T tangential_dist_y = p[0] * (r2 + T(2.0) * yp*yp) + T(2.0) * p[1] * xp*yp;

		T u = xp * radial_dist + tangential_dist_x;
		T v = yp * radial_dist + tangential_dist_y;

		// projected point position
		T predicted_x = f[0] * u + c[0];
		T predicted_y = f[1] * v + c[1];
		// error
		T dx = predicted_x - observed_x;
		T dy = predicted_y - observed_y;
		// output
		residuals[0] = dx;
		residuals[1] = dy;
		return true;
	}
	static ceres::CostFunction* Create(double observed_x, double observed_y) {
		ReprojectionError *re = new ReprojectionError(observed_x, observed_y);
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 15, 3>(re));
	}
};

BundleAdjustmentMultiple::BundleAdjustmentMultiple() {

}
int BundleAdjustmentMultiple::DoWork() {
	google::InitGoogleLogging("HJP");

	// load bundle adjustment dataset, generated from PyCharm
	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/datasets/allposes/dataset_bundle_adjustment_allposes.txt";
	BundleAdjustmentProblem bundle_adjustment_prob;
	if (!bundle_adjustment_prob.LoadDataset(dataset_file_path)) {
		std::cerr << "ERROR: unable to open file " << dataset_file_path << "\n";
		std::getchar();
		return 1;
	}
	else {
		cout << "Done parsing: " << dataset_file_path << endl;
	}


	// load camera parameters from xml
	const char* camparam_txt_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/OutputForCeres/CalibGlobalWorldCentered/camera_parameters.txt";
	if (!bundle_adjustment_prob.LoadCameraParameters(camparam_txt_path)) {
		std::cerr << "ERROR: camera parameters failed to load: " << camparam_txt_path << endl;
		std::getchar();
		return 1;
	}

	// prepare ceres
	std::ofstream output_txt;
	output_txt.open("output/bundleadjustment_output_allposes_huberloss.txt");
	int min_frame = 0;
	int max_frame = bundle_adjustment_prob.num_frames();
	//int max_frame = min_frame + 3;

	// Configure the loss function.
	// delta: https://en.wikipedia.org/wiki/Huber_loss
	const double delta = 1.0;
	ceres::LossFunction* loss = new ceres::HuberLoss(delta);

	// export initial camera parameters
	output_txt << max_frame << "\n";
	double* cp;
	for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
		cp = bundle_adjustment_prob.camera_parameters(cam_idx);
		for (int i = 0; i < 15; i++) {
			output_txt << cp[i] << " ";
		}
	}
	output_txt << "\n";
	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		output_txt << frame_idx << " " << bundle_adjustment_prob.img_names_[frame_idx] << " ";
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			output_txt << bundle_adjustment_prob.chb_detected(frame_idx, cam_idx) << " ";
		}
		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			double * world_point_ptr = bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx);
			output_txt << world_point_ptr[0] << " " << world_point_ptr[1] << " " << world_point_ptr[2] << " ";
		}
		output_txt << "\n";
	}


	// main ceres
	ceres::Problem problem;
	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
					/*
					if (frame_idx == 100 && cam_idx == 0) {
						cout << "frame[" << frame_idx << "], detected: "<< bundle_adjustment_prob.chb_detected(frame_idx, cam_idx) << ", cam[" << cam_idx << "] img("
							<< bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 0) << ", "
							<< bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 1) << ") -> world_point["
							<< world_point_idx << "] = corner("
							<< bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx)[0] << ", "
							<< bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx)[1] << ", "
							<< bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx)[2] << ")" << endl;
					}
					*/
					ceres::CostFunction* cost_function = ReprojectionError::Create(bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 0), bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 1));
					problem.AddResidualBlock(cost_function, loss, bundle_adjustment_prob.camera_parameters(cam_idx), bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx));
				}
			} // for each camera ends
		} // for each world point ends
	} // for each frame ends





	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = 1;
	options.max_num_iterations = 100;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	cout << summary.FullReport() << endl;
	//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << endl;


	// export camera parameters
	for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
		cp = bundle_adjustment_prob.camera_parameters(cam_idx);
		for (int i = 0; i < 15; i++) {
			output_txt << cp[i] << " ";
		}
	}
	output_txt << "\n";
	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		output_txt << frame_idx << " " << bundle_adjustment_prob.img_names_[frame_idx] << " ";
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			output_txt << bundle_adjustment_prob.chb_detected(frame_idx, cam_idx) << " ";
		}
		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			double * world_point_ptr = bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx);
			output_txt << world_point_ptr[0] << " " << world_point_ptr[1] << " " << world_point_ptr[2] << " ";
		}
		output_txt << "\n";
	}
	output_txt << summary.initial_cost << " ";
	output_txt << summary.final_cost << " ";

	output_txt.close();
	printf("\n>> Complete\n");
	std::getchar();
	return 0;
}