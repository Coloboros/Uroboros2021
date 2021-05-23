#%%

import os
import shutil
from source.common import *
from source.settings import MEDIA_DIR
from source.sensers.pcd_sens.sens import Sensor


# Директории файлов с сенсоров
stereo_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_stereo')
tof_dir = os.path.join(MEDIA_DIR, 'point_cloud_train', 'clouds_tof')

# Списки файлов с сенсоров
fns_stereo = get_clouds_names_from_dir(stereo_dir)
fns_tof = get_clouds_names_from_dir(tof_dir)

# Нахождение и сохранение результатов
for i in range(len(fns_tof)):
    Sensor(fns_tof[i]).proc().save_file()

# Удаление временной папки
if os.path.isdir(os.path.join(MEDIA_DIR, 'tmp')):
    shutil.rmtree(os.path.join(MEDIA_DIR, 'tmp'))
