# Configuration file for the Sphinx documentation builder.
import os
import sys
from shutil import copytree, rmtree

# 프로젝트 루트 경로 설정
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

# delto_3f_driver 패키지 경로 추가
sys.path.insert(0, os.path.join(project_root, 'delto_3f_driver'))
sys.path.insert(0, os.path.join(project_root, 'delto_2f_driver'))


# -- Project information -----------------------------------------------------
project = 'DELTO ROS2'
copyright = '2025, TESOLLO'
author = 'TESOLLO'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc', 
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx_copybutton',
]

templates_path = ['_templates']
exclude_patterns = ['build', 'Thumbs.db', '.DS_Store']

# 정적 파일과 리소스 경로 설정
html_static_path = ['_static']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_theme_path = ['_themes', ]
html_theme_options = {
    'navigation_depth': 6,
    
}
html_scaled_image_link = False
autodoc_mock_imports = [
    "rclpy",
    "std_msgs",
    "sensor_msgs",
    "trajectory_msgs",
    "control_msgs",
    "socket",
    "numpy",
    "math",
    "crcmod.predefined",
    "pymodbus",
    "ros_gz_interfaces",
    "delto_utility",
    # 필요 시 추가
]
# 프로젝트 설정 파일 (conf.py)

# 로고 설정
html_logo = "_static/tesollo-ros.svg"

# Build 완료 후 resource 폴더 복사
def setup(app):
    app.connect('build-finished', copy_resource_files)

def copy_resource_files(app, exc):
    if exc is None and app.builder.format == 'html':
        src = os.path.join(os.path.dirname(__file__), 'resource')
        dst = os.path.join(app.outdir, 'resource')
        
        # 이미 존재하는 경우 삭제
        if os.path.exists(dst):
            rmtree(dst)
            
        # 복사 실행    
        copytree(src, dst)
