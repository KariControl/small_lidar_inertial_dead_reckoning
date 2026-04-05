# Third-party components

## small_gicp
- Fork used: https://github.com/KariControl/small_gicp.git
- Commit synchronized from local folder: [master 73b166a] Sync local contents from workspace
 24 files changed, 3519 deletions(-)
 delete mode 100644 docker/Dockerfile.pytest.gcc
 delete mode 100644 docker/Dockerfile.pytest.llvm
 delete mode 100644 docs/Doxyfile
 delete mode 100644 docs/Makefile
 delete mode 100644 docs/assets/downsampling_comp.png
 delete mode 100644 docs/assets/downsampling_threads.png
 delete mode 100644 docs/assets/kdtree_time.png
 delete mode 100644 docs/assets/odometry_native.png
 delete mode 100644 docs/assets/odometry_time.png
 delete mode 100644 docs/conf.py
 delete mode 100644 docs/index.md
 delete mode 100644 docs/index.rst
 delete mode 100644 docs/paper.bib
 delete mode 100644 docs/paper.md
 delete mode 100644 docs/small_gicp.rst
 delete mode 100644 scripts/plot_downsampling.py
 delete mode 100644 scripts/plot_kdtree.py
 delete mode 100644 scripts/plot_odometry.py
 delete mode 100644 scripts/plot_odometry_accuracy.py
 delete mode 100644 scripts/plot_odometry_native.py
 delete mode 100755 scripts/run_downsampling_benchmark.sh
 delete mode 100755 scripts/run_kdtree_benchmark.sh
 delete mode 100755 scripts/run_odometry_benchmark.sh
 delete mode 100755 scripts/run_odometry_benchmark_native.sh
branch 'master' set up to track 'origin/master'.
73b166a1dea4e0c9b0bec7e4bbc49bd3fded5e20
- License: see src/small_gicp/LICENSE

## ndt_omp
- Fork used: https://github.com/KariControl/ndt_omp.git
- Commit synchronized from local folder: [tier4/main ce6e854] Sync local contents from workspace
 10 files changed, 1357 deletions(-)
 delete mode 100644 docker/Dockerfile_gcc
 delete mode 100644 docker/Dockerfile_llvm
 delete mode 100644 regression_test_data/.gitignore
 delete mode 100644 regression_test_data/reference/result.csv
 delete mode 100644 script/compare_regression_test_result.py
 delete mode 100644 script/convert_rosbag_to_test_data.py
 delete mode 100755 script/download_data.sh
 delete mode 100755 script/execute_check_covariance.sh
 delete mode 100755 script/execute_regression_test.sh
 delete mode 100644 script/plot_covariance.py
branch 'tier4/main' set up to track 'origin/tier4/main'.
ce6e85475722e92cc63b4aa5d821dda4b36aee6c
- License: see src/ndt_omp/LICENSE
