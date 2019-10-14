#include "worker.h"


Worker::Worker() {
}

int Worker::DoWork() {
	return 1;
}
int Worker::LoadMeasurementExclusion(const char* txt_path, bool* (&excluded_out)) {
	FILE* txt_fptr = fopen(txt_path, "r");
	if (txt_fptr == NULL) {
		std::cerr << "ERROR: unable to open file: " << txt_path << "\n";
		return 0;
	}

	int num_cycles, num_points, num_frames, is_chb_constrained;
	fscanf(txt_fptr, "%d", &num_cycles);
	fscanf(txt_fptr, "%d", &num_points);
	fscanf(txt_fptr, "%d", &num_frames);
	fscanf(txt_fptr, "%d", &is_chb_constrained);

	
	excluded_out = new bool[num_frames*NUM_CAMERA*num_points];

	int frame_idx_read, cam_idx_read, chb_detected_read;
	for (int cyc_idx = 0; cyc_idx < num_cycles; cyc_idx++) {
		int num_excluded = 0;
		int num_used, num_already_excluded, num_newly_excluded;
		fscanf(txt_fptr, "%d", &num_used);
		fscanf(txt_fptr, "%d", &num_already_excluded);
		fscanf(txt_fptr, "%d", &num_newly_excluded);

		for (int frame_idx = 0; frame_idx < num_frames; frame_idx++) {
			fscanf(txt_fptr, "%d", &frame_idx_read);
			char* img_name[5];
			fscanf(txt_fptr, "%s", &img_name);
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				fscanf(txt_fptr, "%d", &cam_idx_read);
				fscanf(txt_fptr, "%d", &chb_detected_read);
				if (chb_detected_read == 1) {
					for (int p_idx = 0; p_idx < num_points; p_idx++) {
						bool excluded;
						fscanf(txt_fptr, "%d", &excluded);
						if (cyc_idx == num_cycles - 1) {
							excluded_out[(frame_idx*NUM_CAMERA*num_points) + (cam_idx*num_points) + p_idx] = (bool)excluded;
							num_excluded += excluded;
						}
					}
				}
			}
		}

		if (num_newly_excluded == num_excluded) {
			printf("  cycle[%d]: sanity check(Pass), ", cyc_idx);
		}
		else {
			printf("  cycle[%d]: sanity check(Fail), ", cyc_idx);
			std::getchar();
			return 0;
		}

		float excluded_percent = float(num_excluded) / (float(num_used) + float(num_already_excluded)) * 100.f;
		printf("%d (%.2f%%) excluded.\n", num_excluded, excluded_percent);
	}
	return 1;
}

int Worker::LoadCameraParameters(const char* xml_path, CameraParameters* (&cam_params)[NUM_CAMERA]) {
	FILE* camparam_fptr = fopen(xml_path, "r");
	if (camparam_fptr == NULL) {
		std::cerr << "ERROR: unable to open file: " << xml_path << "\n";
		std::getchar();
		return 0;
	}

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

		cam_params[cam_idx] = new CameraParameters(cam_idx_confirm, ax1, ax2, ax3, t1, t2, t3, fx, fy, c1, c2, k1, k2, p1, p2, k3);
	}
	cout << "Done parsing: " << xml_path << endl;
	return 1;
}