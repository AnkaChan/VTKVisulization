#include "bundle_adjustment_single.h"
#define LEN_IMAGE_NAME 5
#define NUM_POINTS 88

static bool* g_chb_detected; // 8 x num_frames
class BundleAdjustmentProblemSingle {
public:
	~BundleAdjustmentProblemSingle() {
		delete[] g_chb_detected;
		delete[] image_point_measurements_;
		delete[] img_names_;
	}
	double image_point_measurements(int frame_idx, int cam_idx, int img_idx, int dim) {
		return image_point_measurements_[frame_idx * NUM_CAMERA * NUM_POINTS * 2 + cam_idx * NUM_POINTS * 2 + img_idx * 2 + dim];
	}
	const bool* chb_detected() const {
		return g_chb_detected;
	}
	int num_frames() const {
		return num_frames_;
	}
	double* world_point_estimates(int frame_idx, int cam_idx, int point_idx) {
		// return 88 world points for a frame
		return world_points + ((frame_idx * NUM_CAMERA * NUM_POINTS * 3) + (cam_idx * NUM_POINTS * 3) + point_idx * 3);
	}
	bool LoadDataset(const char* filename) {
		srand(time(NULL));
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			printf("File pointer null: %s\n", filename);
			std::getchar();
			return false;
		}

		FscanfOrDie(fptr, "%d", &num_frames_);
		g_chb_detected = new bool[NUM_CAMERA * num_frames_];
		image_point_measurements_ = new double[NUM_CAMERA * NUM_POINTS * 2 * num_frames_](); // initialize to 0s
		img_names_ = (char **)malloc((LEN_IMAGE_NAME + 1) * num_frames_ * sizeof(char*));
		world_points = new double[num_frames_* NUM_POINTS * NUM_CAMERA * 3];

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
				g_chb_detected[frame_idx * NUM_CAMERA + cam_idx] = (chb_detected == 1);

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
				FscanfOrDie(fptr, "%lf", &X);
				FscanfOrDie(fptr, "%lf", &Y);
				FscanfOrDie(fptr, "%lf", &Z);

				for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
					/* generate secret number between 1 and 100: */
					int rand_int = rand() % 100 + 1;
					double rand_double = double(rand_int);
					world_points[(frame_idx*NUM_CAMERA*NUM_POINTS * 3) + (cam_idx * NUM_POINTS * 3) + (p_idx * 3) + 0] = X + rand_double;
					world_points[(frame_idx*NUM_CAMERA*NUM_POINTS * 3) + (cam_idx * NUM_POINTS * 3) + (p_idx * 3) + 1] = Y + rand_double;
					world_points[(frame_idx*NUM_CAMERA*NUM_POINTS * 3) + (cam_idx * NUM_POINTS * 3) + (p_idx * 3) + 2] = Z + rand_double;
				}
			}
		}
#if 0
		// check data input
		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			if (frame_idx < 160 || frame_idx > 170) continue;
			printf("frame[%d] image_name: %s ===============================\n", frame_idx, img_names_[frame_idx]);
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				bool dtd = g_chb_detected[frame_idx * NUM_CAMERA + cam_idx];
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
			cout << frame_idx << "| " << world_points[frame_idx*NUM_POINTS * 3 + 0] << ", " << world_points[frame_idx*NUM_POINTS * 3 + 1] << ", " << world_points[frame_idx*NUM_POINTS * 3 + 2] << endl;
		}
#endif
		return true;
	}
	char** img_names_;

	// world point estimates
	double *world_points;
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

	int curr_camera_idx_;

	double* image_point_measurements_; // 2 x 8 x num_frames
	double world_point_estimates_; // num_frames_ x 88 x 3
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
		T radial_dist = 1.0 + r2 * (k[0] + k[1] * r2 + k[2]*r2*r2);
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

BundleAdjustmentSingle::BundleAdjustmentSingle() {

}
int BundleAdjustmentSingle::DoWork() {
	google::InitGoogleLogging("HJP");

	// load bundle adjustment dataset, generated from PyCharm
	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib/datasets/allposes/_dataset_bundle_adjustment_final.txt";
	BundleAdjustmentProblemSingle bundle_adjustment_prob;
	if (!bundle_adjustment_prob.LoadDataset(dataset_file_path)) {
		std::cerr << "ERROR: unable to open file " << dataset_file_path << "\n";
		std::getchar();
		return 1;
	}
	else {
		cout << "Done parsing: " << dataset_file_path << endl;
	}


	// load camera parameters from xml
	CameraParameters *cam_params_default[NUM_CAMERA];
	const char* camparam_txt_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib/_camera_parameters.txt";
	int res = LoadCameraParameters(camparam_txt_path, cam_params_default);


	// prepare ceres
	std::ofstream output_txt;
	output_txt.open("bundle_adj_result.txt");
	int min_frame = 0;
	int max_frame = bundle_adjustment_prob.num_frames();
	//int max_frame = min_frame + 3;


	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		// intialization
		CameraParameters *cam_params[NUM_CAMERA];
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			double *cp = cam_params_default[cam_idx]->GetDataArray();
			cam_params[cam_idx] = new CameraParameters(cam_idx, cp[0], cp[1], cp[2], cp[3], cp[4], cp[5], cp[6], cp[7], cp[8], cp[9], cp[10], cp[11], cp[12], cp[13], cp[14]);
		}


		// export intial values to txt file - for vtk
		output_txt << frame_idx <<" " << bundle_adjustment_prob.img_names_[frame_idx] << " ";
		printf("[%s.pgm] ", bundle_adjustment_prob.img_names_[frame_idx]);
		for (int c = 0; c < NUM_CAMERA; c++) {
			output_txt << g_chb_detected[frame_idx * NUM_CAMERA + c] << " ";
		}
		for (int c = 0; c < NUM_CAMERA; c++) {
			double * cp = cam_params[c]->GetDataArray();
			for (int i = 0; i < 15; i++) {
				output_txt << cp[i] << " ";
			}
		}
		for (int pi = 0; pi < NUM_POINTS; pi++) {
			double * world_point_ptr = bundle_adjustment_prob.world_point_estimates(frame_idx, 0, pi);
			output_txt << world_point_ptr[0] << " " << world_point_ptr[1] << " " << world_point_ptr[2] << " ";
		}
		output_txt << "_ ";



		int cam_used = 0;
		ceres::Problem problem;
		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (g_chb_detected[frame_idx * NUM_CAMERA + cam_idx]) {
					ceres::CostFunction* cost_function = ReprojectionError::Create(bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 0), bundle_adjustment_prob.image_point_measurements(frame_idx, cam_idx, world_point_idx, 1));
					problem.AddResidualBlock(cost_function, NULL, cam_params[cam_idx]->GetDataArray(), bundle_adjustment_prob.world_point_estimates(frame_idx, cam_idx, world_point_idx));
					cam_used++;
				}
			} // for each camera ends
		} // for each world point ends


		if (int(cam_used / NUM_POINTS) > 0) {
			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_SCHUR;
			options.minimizer_progress_to_stdout = 0;
			options.max_num_iterations = 100;

			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//cout << summary.FullReport() << endl;
			//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
			//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << endl;
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				double * cp = cam_params[cam_idx]->GetDataArray();
				for (int i = 0; i < 15; i++) {
					output_txt << cp[i] << " ";
				}
			}

			for (int pi = 0; pi < NUM_POINTS; pi++) {
				double * world_point_ptr = bundle_adjustment_prob.world_point_estimates(frame_idx, 0, pi);
				output_txt << world_point_ptr[0] << " " << world_point_ptr[1] << " " << world_point_ptr[2] << " ";
			}
			output_txt << summary.initial_cost << " ";
			output_txt << summary.final_cost << " ";
			output_txt << "\n";
		}

		// clean up
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			delete cam_params[cam_idx];
		}
	} // for each frame ends
	output_txt.close();
	printf("\n>> Complete\n");
	std::getchar();

	// clean up
	for (int c = 0; c < NUM_CAMERA; c++) {
		delete cam_params_default[c];
	}
	std::getchar();

	return 0;
}
#if 0
#endif