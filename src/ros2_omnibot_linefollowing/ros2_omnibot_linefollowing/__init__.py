try:
            with np.load(self.npz_matrix_path) as data:
                camera_matrix = data['camMatrix']
                dist_coeffs = data['distCoef']
        
        except Exception as e:
            print(f'calibration failed: {e}')
            exit()