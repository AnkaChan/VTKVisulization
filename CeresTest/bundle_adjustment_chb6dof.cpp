#define  _CRT_SECURE_NO_WARNINGS
#include <math.h>
#include "bundle_adjustment_chb6dof.h"

#define LEN_IMAGE_NAME 5
#define NUM_POINTS 88

#define K3OFF
#define LOSS_FUNC
class LossFunction {
public:
	virtual void Evaluate(double s, double out[3]) const = 0;
};

class BundleAdjustmentProblemChb6Dof {
public:
	~BundleAdjustmentProblemChb6Dof() {
		delete[] chb_detected_;
		delete[] image_point_measurements_;
		delete[] img_names_;
		delete[] cam_params_;
		delete[] chb_config_;
	}
	double* cam_params(int cam_idx) {
		// return first index of camera parameters array for the camera index
		return cam_params_ + cam_idx * 15;
	}

	double* chb_config(int frame_idx) {
		return chb_config_ + frame_idx * 6;
	}
	double* image_point_measurements_arr(int frame_idx, int cam_idx) {
		// giving the first index of 88 2d image points arr
		return image_point_measurements_ + frame_idx * NUM_CAMERA * NUM_POINTS * 2 + cam_idx * NUM_POINTS * 2;
	}
	const bool chb_detected(int frame_idx, int cam_idx) const {
		return chb_detected_[frame_idx * NUM_CAMERA + cam_idx];
	}
	int num_frames() const {
		return num_frames_;
	}
	bool LoadCameraParameters(const char* xml_path) {
		FILE* camparam_fptr = fopen(xml_path, "r");
		if (camparam_fptr == NULL) {
			std::cerr << "ERROR: unable to open file: " << xml_path << "\n";
			std::getchar();
			return false;
		}

		cam_params_ = new double[15 * NUM_CAMERA];
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			double ax1, ax2, ax3, t1, t2, t3, fx, fy, c1, c2, k1, k2, p1, p2, k3;
			int cam_idx_confirm;
			fscanf(camparam_fptr, "%d", &cam_idx_confirm);

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

			cam_params_[cam_idx * 15 + 0] = ax1;
			cam_params_[cam_idx * 15 + 1] = ax2;
			cam_params_[cam_idx * 15 + 2] = ax3;

			cam_params_[cam_idx * 15 + 3] = t1;
			cam_params_[cam_idx * 15 + 4] = t2;
			cam_params_[cam_idx * 15 + 5] = t3;

			cam_params_[cam_idx * 15 + 6] = fx;
			cam_params_[cam_idx * 15 + 7] = fy;

			cam_params_[cam_idx * 15 + 8] = c1;
			cam_params_[cam_idx * 15 + 9] = c2;

			cam_params_[cam_idx * 15 + 10] = k1;
			cam_params_[cam_idx * 15 + 11] = k2;
			cam_params_[cam_idx * 15 + 12] = p1;
			cam_params_[cam_idx * 15 + 13] = p2;
			cam_params_[cam_idx * 15 + 14] = k3;
		}
		cout << "Done parsing: " << xml_path << endl;
		return true;
	}
	bool LoadDataset(const char* filename) {
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			return false;
		}

		FscanfOrDie(fptr, "%d", &num_frames_);

		chb_detected_ = new bool[NUM_CAMERA * num_frames_];
		image_point_measurements_ = new double[NUM_CAMERA * NUM_POINTS * 2 * num_frames_](); // initialize to 0s
		img_names_ = (char **)malloc((LEN_IMAGE_NAME + 1) * num_frames_ * sizeof(char*));
		chb_config_ = new double[num_frames_ * 6];

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
			FscanfOrDie(fptr, "%s", &img_name_);

			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 0]);
			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 1]);
			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 2]);
			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 3]);
			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 4]);
			FscanfOrDie(fptr, "%lf", &chb_config_[frame_idx * 6 + 5]);
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
	double *cam_params_; // 15*8
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

	double *chb_config_;
};

struct ReprojectionError {
	static double chb_points[NUM_POINTS*3];
	double obsv_points[NUM_POINTS*2];

	ReprojectionError(double *observed_points) {
		// observed_points -> 2*88 size
		for (int i = 0; i < NUM_POINTS; i++) {
			obsv_points[i * 2 + 0] = observed_points[i * 2 + 0];
			obsv_points[i * 2 + 1] = observed_points[i * 2 + 1];
		}
	}
	

	template<typename T>
	bool operator()(const T* const camera, const T* const chbR, const T* const chbT, T* residuals) const {
		// extract camera parameters
		const T ang_axis[3] = { camera[0], camera[1], camera[2] };
		const T trans[3] = { camera[3], camera[4], camera[5] };
		const T f[2] = { camera[6], camera[7] };
		const T c[2] = { camera[8], camera[9] };
		const T k[3] = { camera[10], camera[11], camera[14] }; // k3 is excluded here. camera[14]
		const T p[2] = { camera[12], camera[13] };


		// chb parameters
		const T chb_ang_axis[3] = { chbR[0], chbR[1], chbR[2] };
		const T chb_trans[3] = { chbT[0], chbT[1], chbT[2] };

		// world_points
		for (int i = 0; i < NUM_POINTS; i++) {
			// update checkerboard points
			const T chb_point[3] = { T(chb_points[i * 3]), T(chb_points[i * 3 + 1]), T(chb_points[i * 3 + 2]) };
			T world_point[3];
			ceres::AngleAxisRotatePoint(chb_ang_axis, chb_point, world_point);

			// translation
			world_point[0] += chb_trans[0];
			world_point[1] += chb_trans[1];
			world_point[2] += chb_trans[2];

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
			T dx = predicted_x - obsv_points[i * 2 + 0];
			T dy = predicted_y - obsv_points[i * 2 + 1];

			// output
			residuals[2 * i + 0] = dx;
			residuals[2 * i + 1] = dy;
		}
		return true;
	}

	static ceres::CostFunction* Create(double *observed_points) {
		ReprojectionError *re = new ReprojectionError(observed_points);
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 176, 15, 3, 3>(re));
	}
};
double ReprojectionError::chb_points[NUM_POINTS * 3] = { -0, 0, 0,
-60, 0, 0,
-120, 0, 0,
-180, 0, 0,
-240, 0, 0,
-300, 0, 0,
-360, 0, 0,
-420, 0, 0,
-480, 0, 0,
-540, 0, 0,
-600, 0, 0,
-0, 60, 0,
-60, 60, 0,
-120, 60, 0,
-180, 60, 0,
-240, 60, 0,
-300, 60, 0,
-360, 60, 0,
-420, 60, 0,
-480, 60, 0,
-540, 60, 0,
-600, 60, 0,
-0, 120, 0,
-60, 120, 0,
-120, 120, 0,
-180, 120, 0,
-240, 120, 0,
-300, 120, 0,
-360, 120, 0,
-420, 120, 0,
-480, 120, 0,
-540, 120, 0,
-600, 120, 0,
-0, 180, 0,
-60, 180, 0,
-120, 180, 0,
-180, 180, 0,
-240, 180, 0,
-300, 180, 0,
-360, 180, 0,
-420, 180, 0,
-480, 180, 0,
-540, 180, 0,
-600, 180, 0,
-0, 240, 0,
-60, 240, 0,
-120, 240, 0,
-180, 240, 0,
-240, 240, 0,
-300, 240, 0,
-360, 240, 0,
-420, 240, 0,
-480, 240, 0,
-540, 240, 0,
-600, 240, 0,
-0, 300, 0,
-60, 300, 0,
-120, 300, 0,
-180, 300, 0,
-240, 300, 0,
-300, 300, 0,
-360, 300, 0,
-420, 300, 0,
-480, 300, 0,
-540, 300, 0,
-600, 300, 0,
-0, 360, 0,
-60, 360, 0,
-120, 360, 0,
-180, 360, 0,
-240, 360, 0,
-300, 360, 0,
-360, 360, 0,
-420, 360, 0,
-480, 360, 0,
-540, 360, 0,
-600, 360, 0,
-0, 420, 0,
-60, 420, 0,
-120, 420, 0,
-180, 420, 0,
-240, 420, 0,
-300, 420, 0,
-360, 420, 0,
-420, 420, 0,
-480, 420, 0,
-540, 420, 0,
-600, 420, 0, };


BundleAdjustmentChb6Dof::BundleAdjustmentChb6Dof() {

}
int BundleAdjustmentChb6Dof::DoWork() {
	google::InitGoogleLogging("HJP");

	const int num_cycle = 1;

	// master output
	std::ofstream output_txt;
	output_txt.open("output/bundleadjustment_output7_chb6dof.txt");

	// load bundle adjustment dataset, generated from PyCharm
	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/datasets/allposes/dataset_bundle_adjustment_chb6dof.txt";
	BundleAdjustmentProblemChb6Dof bundle_adjustment_prob;
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



	const bool is_chb_constrained = true;
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
				cp = bundle_adjustment_prob.cam_params(cam_idx);
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
					output_txt << -7 << " " << -7 << " " << -7 << " ";
				}
				output_txt << "\n";
			}
		}


		// main ceres
		ceres::Problem problem;
		for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (bundle_adjustment_prob.chb_detected(frame_idx, cam_idx)) {

					ceres::CostFunction* cost_function = ReprojectionError::Create(bundle_adjustment_prob.image_point_measurements_arr(frame_idx, cam_idx));
					//double* chb = bundle_adjustment_prob.chb_config(frame_idx);
					//printf("frame[%d] camera[%d]: (%f, %f, %f)\n", frame_idx, cam_idx, chb[0], chb[1], chb[2]);
					double *chb = bundle_adjustment_prob.chb_config(frame_idx);
					problem.AddResidualBlock(cost_function, loss, bundle_adjustment_prob.cam_params(cam_idx), chb, chb+3);
				} // for each world point ends
			} // for each camera ends
		} // for each frame ends



		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		options.minimizer_progress_to_stdout = 1;
		options.max_num_iterations = 100;
		//options.num_linear_solver_threads = 1;
		//options.num_threads = 6;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		cout << summary.FullReport() << endl;
		//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << endl;


		if (cycle_idx == num_cycle - 1) {
			double* cp;
			// export camera parameters
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				cp = bundle_adjustment_prob.cam_params(cam_idx);
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
					output_txt << -7 << " " << -7 << " " << -7 << " ";
				}
				output_txt << "\n";
			}


			for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
				output_txt << frame_idx << " " << bundle_adjustment_prob.img_names_[frame_idx] << " ";
				for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
					output_txt << bundle_adjustment_prob.chb_detected(frame_idx, cam_idx) << " ";
				}
				double *chb = bundle_adjustment_prob.chb_config(frame_idx);
				output_txt << chb[0] << " " << chb[1] << " " << chb[2] << " " << chb[3] << " " << chb[4] << " " << chb[5] << " ";
				output_txt << "\n";
			}

			output_txt << summary.initial_cost << " ";
			output_txt << summary.final_cost << " ";
			output_txt.close();
		}
	} // cycle ends


	printf("\n>> Complete\n");
	std::getchar();
	return 0;
}