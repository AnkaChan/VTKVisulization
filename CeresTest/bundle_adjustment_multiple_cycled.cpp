#include "bundle_adjustment_multiple_cycled.h"
#define LEN_IMAGE_NAME 5
#define NUM_POINTS 88

#define K3OFF
//#define RANDOMIZE
//#define LOSS_FUNC
class BundleAdjustmentProblemCycled {
public:
	~BundleAdjustmentProblemCycled() {
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

	double *world_points; // num_frames * 88 * 3
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

BundleAdjustmentMultipleCycled::BundleAdjustmentMultipleCycled() {

}
double BundleAdjustmentMultipleCycled::ComputeReprojectionError(double* camera, double* world_point, double measured_x, double measured_y) {
	const double ang_axis[3] = { camera[0], camera[1], camera[2] };
	const double trans[3] = { camera[3], camera[4], camera[5] };
	const double f[2] = { camera[6], camera[7] };
	const double c[2] = { camera[8], camera[9] };
	const double k[3] = { camera[10], camera[11], camera[14] }; // k3 is excluded here. camera[14]
	const double p[2] = { camera[12], camera[13] };

	double cam_point[3];
	// angle-axis rotation
	ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

	// translation
	cam_point[0] += trans[0];
	cam_point[1] += trans[1];
	cam_point[2] += trans[2];

	// center of distortion
	double xp = cam_point[0] / cam_point[2];
	double yp = cam_point[1] / cam_point[2];

	// distortions
	double r2 = xp * xp + yp * yp;
#ifdef K3OFF
	double radial_dist = 1.0 + r2 * (k[0] + k[1] * r2);
#else
	double radial_dist = 1.0 + r2 * (k[0] + k[1] * r2 + k[2] * r2*r2);
#endif
	double tangential_dist_x = 2.0 * p[0] * xp*yp + p[1] * (r2 + 2.0 * xp*xp);
	double tangential_dist_y = p[0] * (r2 + 2.0 * yp*yp) + 2.0 * p[1] * xp*yp;

	double u = xp * radial_dist + tangential_dist_x;
	double v = yp * radial_dist + tangential_dist_y;

	// projected point position
	double predicted_x = f[0] * u + c[0];
	double predicted_y = f[1] * v + c[1];
	
	// error
	double dx = predicted_x - measured_x;
	double dy = predicted_y - measured_y;

	// output
	return sqrt(dx * dx + dy * dy);
}
int BundleAdjustmentMultipleCycled::DoWork() {
	google::InitGoogleLogging("HJP");

	const int num_cycle = 1;

	// master output
	std::ofstream output_txt;
	output_txt.open("output/bundleadjustment5_output.txt");

	// reprojection output
	std::ofstream output_err;
	output_err.open("output/output_err.txt");

	// image usages
	std::ofstream output_excluded_measurements;
	output_excluded_measurements.open("output/output_excluded_measurements5.txt");

	double *reprojection_errors = NULL;
	bool *excluded_measurements = NULL;
	double cutoff_values[NUM_CAMERA];
	for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
		cutoff_values[cam_idx] = std::numeric_limits<double>::infinity();
	}







	// load bundle adjustment dataset, generated from PyCharm
	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/datasets/allposes/dataset_bundle_adjustment_allposes.txt";
	BundleAdjustmentProblemCycled bundle_adjustment_prob;
	if (!bundle_adjustment_prob.LoadDataset(dataset_file_path)) {
		std::cerr << "ERROR: unable to open file " << dataset_file_path << "\n";
		std::getchar();
		return 1;
	}
	else {
		cout << "Done parsing: " << dataset_file_path << endl;
	}
	int min_frame = 0;
	int max_frame = bundle_adjustment_prob.num_frames();


	// load camera parameters from xml
	const char* camparam_txt_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/OutputForCeres/CalibGlobalWorldCentered/camera_parameters.txt";
	if (!bundle_adjustment_prob.LoadCameraParameters(camparam_txt_path)) {
		std::cerr << "ERROR: camera parameters failed to load: " << camparam_txt_path << endl;
		std::getchar();
		return 1;
	}


	bool is_chb_constrained = false;
	output_err << num_cycle << " " << NUM_POINTS << " " << max_frame << " " << is_chb_constrained << "\n";
	output_excluded_measurements << num_cycle << " " << NUM_POINTS << " " << max_frame << " " << is_chb_constrained << "\n";
	for (int cycle_idx = 0; cycle_idx < num_cycle; cycle_idx++) {
		// Configure the loss function.
#ifdef LOSS_FUNC
	// delta: https://en.wikipedia.org/wiki/Huber_loss
		const double delta = 1.0;
		ceres::LossFunction* loss = new ceres::HuberLoss(delta);
#else
		ceres::LossFunction* loss = NULL;
#endif

		if (cycle_idx == 0) {
			// export initial camera parameters
			output_txt << max_frame << " " << is_chb_constrained << "\n";
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
		}
		if (reprojection_errors == NULL) {
			printf("* reprojection_errors initialized w/ size %d\n", max_frame * NUM_CAMERA * NUM_POINTS);
			reprojection_errors = new double[max_frame * NUM_CAMERA * NUM_POINTS]{ 0 };
		}
		if (excluded_measurements == NULL) {
			printf("* excluded_measurements initialized w/ size %d\n", max_frame * NUM_CAMERA * NUM_POINTS);
			excluded_measurements = new bool[max_frame * NUM_CAMERA * NUM_POINTS]{ false };
		}



		// main ceres
		ceres::Problem problem;
		int points_used = 0;
		int points_newly_excluded = 0;
		int points_already_exclude = 0;
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
				for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
					if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
						double reprojection_err = reprojection_errors[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx * NUM_POINTS + world_point_idx];
						if (reprojection_err < cutoff_values[cam_idx]) {
							ceres::CostFunction* cost_function = ReprojectionError::Create(bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 0), bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 1));
							problem.AddResidualBlock(cost_function, loss, bundle_adjustment_prob.camera_parameters(cam_idx), bundle_adjustment_prob.world_point_estimates(frame_idx, world_point_idx));
							points_used++;
						}
						else {
							if (excluded_measurements[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx * NUM_POINTS + world_point_idx] == false) {
								excluded_measurements[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx * NUM_POINTS + world_point_idx] = true;
								points_newly_excluded++;
							}
							else {
								points_already_exclude++;
							}
						}
					}
				} // for each camera ends
			} // for each world point ends
		} // for each frame ends
		printf("%d number of points used, %d already excluded, %d newly excluded\n", points_used, points_already_exclude, points_newly_excluded);
		output_excluded_measurements << points_used << " " << points_already_exclude << " " << points_newly_excluded << "\n";
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			output_excluded_measurements << frame_idx << " " << bundle_adjustment_prob.img_names_[frame_idx] << " ";
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				output_excluded_measurements << cam_idx << " ";
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
					output_excluded_measurements << "1 ";
					for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
						output_excluded_measurements << excluded_measurements[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx * NUM_POINTS + world_point_idx] << " ";
					}
				}
				else {
					output_excluded_measurements << "0 ";
				}
			}
			output_excluded_measurements << "\n";
		}


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



		// identify outlier measurements from the reprojection errors
		double *reproj_err_every_cam = new double[max_frame*NUM_CAMERA]{ 0 };
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			output_err << frame_idx << " ";
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				output_err << cam_idx << " " << bundle_adjustment_prob.chb_detected(frame_idx, cam_idx) << " ";
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
					double reproj_err_sum = 0;
					double *cam_param = bundle_adjustment_prob.camera_parameters(cam_idx);
					for (int p_idx = 0; p_idx < NUM_POINTS; p_idx++) {
						bool is_excluded = excluded_measurements[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx * NUM_POINTS + p_idx];
						if (!is_excluded) {
							double* world_point = bundle_adjustment_prob.world_point_estimates(frame_idx, p_idx);
							double measured_x = bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, p_idx, 0);
							double measured_y = bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, p_idx, 1);
							double reproj_err = ComputeReprojectionError(cam_param, world_point, measured_x, measured_y);
							reproj_err_sum += reproj_err;
							reprojection_errors[frame_idx * NUM_CAMERA * NUM_POINTS + cam_idx * NUM_POINTS + p_idx] = reproj_err;
							output_err << reproj_err << " ";
						}
						else {
							output_err << "-1" << " "; 
						}
					}
					reproj_err_every_cam[frame_idx*NUM_CAMERA + cam_idx] = reproj_err_sum / NUM_POINTS;
				} // chb detected ends
			} // each camera end
			output_err << "\n";
		} // each frame ends


		
		// sum
		double err_sum_each_cam[NUM_CAMERA] = { 0 };
		int valid_frame_counts[NUM_CAMERA] = { 0 };
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
					err_sum_each_cam[cam_idx] += reproj_err_every_cam[frame_idx*NUM_CAMERA + cam_idx];
					valid_frame_counts[cam_idx] += 1;
				}
			}
		}
		// average
		double err_mean_each_cam[NUM_CAMERA] = { 0 };
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			err_mean_each_cam[cam_idx] = err_sum_each_cam[cam_idx] / valid_frame_counts[cam_idx];
		}
		// stdv
		double err_var_each_cam[NUM_CAMERA] = { 0 };
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {
					double mean = err_mean_each_cam[cam_idx];
					double x = reproj_err_every_cam[frame_idx*NUM_CAMERA + cam_idx];
					err_var_each_cam[cam_idx] += (x - mean)*(x - mean);
				}
			}
		}
		double err_stdv_each_cam[NUM_CAMERA] = { 0 };
		printf("\ncycle[%d] cut-off values-------------------\n", cycle_idx);
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			err_var_each_cam[cam_idx] /= (valid_frame_counts[cam_idx]);
			err_stdv_each_cam[cam_idx] = sqrt(err_var_each_cam[cam_idx]);

			// cut-off values
			static const double stdv_multiplier = 5.0;
			cutoff_values[cam_idx] = err_mean_each_cam[cam_idx] + stdv_multiplier * err_stdv_each_cam[cam_idx];
			printf("cam[%d]: mean(%f)\t stdv(%f)\t cutoff(%f)\n", cam_idx, err_mean_each_cam[cam_idx], err_stdv_each_cam[cam_idx], cutoff_values[cam_idx]);
		}
		printf("\n");

		delete[] reproj_err_every_cam;



		if (cycle_idx == num_cycle-1) {
			double* cp;
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
			output_err.close();
			output_excluded_measurements.close();
		}
	} // cycle ends


	printf("\n>> Complete\n");
	delete[] reprojection_errors;
	delete[] excluded_measurements;

	std::getchar();
	return 0;
}