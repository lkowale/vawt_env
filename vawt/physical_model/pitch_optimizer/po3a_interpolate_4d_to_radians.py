from vawt.physical_model.pitch_optimizer.po3a_interpolate_4d import OptimalPathInterpolate
import time



if __name__ == '__main__':

    start_time = time.time()
    opi = OptimalPathInterpolate('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_10/')
    # opi.plot_grid(3.5)
    opi.indexes_to_radians()
