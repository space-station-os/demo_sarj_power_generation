import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from mpl_toolkits.mplot3d import Axes3D
import glob


def draw_latlon(in_df):

    lat_deg = in_df['latitude']
    lon_deg = in_df['longitude']

    # 日付変更線をまたぐときに線が長くならないように補正
    lon_deg_corrected = lon_deg.copy()
    for i in range(1, len(lon_deg)):
        diff = lon_deg_corrected[i] - lon_deg_corrected[i-1]
        if diff > 180:
            lon_deg_corrected[i:] -= 360
        elif diff < -180:
            lon_deg_corrected[i:] += 360

    # --- 世界地図に軌道を描画 ---
    fig, ax = plt.subplots(figsize=(12, 6), subplot_kw={'projection': ccrs.PlateCarree()})
    ax.set_facecolor('black')

    # 海岸線を明るい緑で描画
    ax.add_feature(cfeature.COASTLINE.with_scale('110m'), edgecolor='lime')

    # 衛星軌道を赤線で描画
    ax.plot(lon_deg_corrected, lat_deg, color='red', linewidth=1.5, transform=ccrs.PlateCarree())

    # 地球全体を表示
    ax.set_extent([-180, 180, -90, 90], crs=ccrs.PlateCarree())

    # 緯度・経度のラベルを左と下に表示
    ax.set_xticks(np.arange(-180, 181, 60), crs=ccrs.PlateCarree())
    ax.set_yticks(np.arange(-90, 91, 30), crs=ccrs.PlateCarree())
    ax.tick_params(labelcolor='white')  # ラベルを白色に
    ax.spines['geo'].set_edgecolor('white')  # 図の枠線を白色に

    ax.set_title('Satellite Ground Track', color='white')
    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
    plt.show()

    # --- 高度 vs 時間 ---
    unix_time = in_df['unix_time']
    altitude = in_df['altitude']  # [m]

    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(unix_time, altitude / 1000, color='red')  # [km]単位で表示
    ax2.set_xlabel('Unix Time [s]')
    ax2.set_ylabel('Altitude [km]')
    ax2.set_title('Altitude over Time')
    ax2.grid(True)
    plt.show()


def draw_motion_radius(in_df):
    # draw the disntance from the center of the earth. != altitude
    unix_time_sr = in_df['simu_time']
    eci_x_sr = in_df['ss_position_eci_x']
    eci_y_sr = in_df['ss_position_eci_y']
    eci_z_sr = in_df['ss_position_eci_z']

    eci_r_sr = np.sqrt(eci_x_sr**2 + eci_y_sr**2 + eci_z_sr**2)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(unix_time_sr, eci_r_sr)
    ax.set_ylabel('Motion Radius [m]')
    ax.set_xlabel('time [s]')
    plt.show()

    return


def draw_3d_orbit(in_df):
    eci_x = in_df['ss_position_eci_x']
    eci_y = in_df['ss_position_eci_y']
    eci_z = in_df['ss_position_eci_z']
    eci_ax = in_df['ss_acceleration_eci_x']
    eci_ay = in_df['ss_acceleration_eci_y']
    eci_az = in_df['ss_acceleration_eci_z']

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1,1,1])

    # --- 地球を半透明の青い球体で描画 ---
    # 地球の半径（平均値）
    earth_radius = 6371e3  # [m]

    # 球面座標を生成
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z, color='blue', alpha=0.3, edgecolor='none')

    # 軌道線を描画
    ax.plot(eci_x, eci_y, eci_z, color='red', linewidth=1.5)

    # 加速度ベクトルを描画（間引き）
    skip = max(1, len(in_df) // 50)
    ax.quiver(
        eci_x[::skip], eci_y[::skip], eci_z[::skip],
        eci_ax[::skip], eci_ay[::skip], eci_az[::skip],
        length=500000, color='red', normalize=True
    )

    # for J2
    ax.view_init(elev=30, azim=45)
    # for air drag
    # ax.view_init(elev=30, azim=135)

    ax.set_xlabel('ECI X [m]')
    ax.set_ylabel('ECI Y [m]')
    ax.set_zlabel('ECI Z [m]')
    ax.set_title('3D Orbit in ECI Frame')
    plt.show()


def draw_soc(in_df: pd.DataFrame):
    unix_time_sr = in_df['simu_time']
    battery_level_sr = in_df['battery_level']
    
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(unix_time_sr, battery_level_sr)
    ax.set_ylabel('Battery Level [Wh]')
    ax.set_xlabel('time [s]')
    plt.show()

    return


def main():
    
    in_filepath_list = sorted(glob.glob('./result_csv/*'))
    in_filepath = in_filepath_list[-1]
    
    in_df = pd.read_csv(in_filepath)
    print(in_df.columns)

    # draw_latlon(in_df)
    # draw_3d_orbit(in_df)
    draw_motion_radius(in_df)
    draw_soc(in_df)
    return


if __name__ == '__main__':
    main()
