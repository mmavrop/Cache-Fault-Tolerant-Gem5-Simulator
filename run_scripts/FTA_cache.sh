#!/bin/bash

##########################################################################
# Use this script in order to run gem5 for multiple cache configurations #
##########################################################################


###---------------------Simulation Configurations--------------------#####
M5_PATH=$PWD"/"
build_dir="build/X86/gem5.opt"
configs_dir=$M5_PATH"configs/common/"
configs_init=$configs_dir"Caches_init.py"
configs_inter=$configs_dir"Caches_inter.py"
configs_write=$configs_dir"Caches.py"
cache_dir=$M5_PATH"src/mem/cache/tags/"
predictor_dir=$M5_PATH"src/mem/cache/predictor/"
output_dir="FTA_cache"
results_path=$M5_PATH"results"
results_dir=$results_path"/"$output_dir

# number of available threads for parallelism
threads=4 

# Memory configurations
l1d_size="32"  # one or more (in kB)
l1i_size="16"  # one or more (in kB)
l1d_assoc="4"  # one or more
l1i_assoc="4"  # one or more
l2_size="1MB"
l2_assoc="8"
mem_size="8GB"

#fault maps
fltmaps_dir=$cache_dir"fault_maps"
fault_map_select="01"                           # run specific fault maps
pfail_select="1E-03"                            # choices: 5E-04, 1E-03, 2E-03 
fta_sub_blocks="8"                              # sub_block choices: 4, 8 
fta_sampling_rate="32"                          # FTA sampling rate
fta_pred_conf="fta_predictor_64_LSB_6_ADDR.hh"  # FTA prediction configurations: src/mem/cache/predictor
fcache_type="DL1"                               # choices: "IL1", "DL1"


#Benchmarks
benchmark_dir="polybench-c-4.2"
poly_datamining="correlation covariance"
poly_blas="gemver gesummv gemm symm syr2k syrk trmm"
poly_kernels="atax" #bicg doitgen 2mm 3mm atax bicg doitgen mvt"
poly_solvers="cholesky durbin gramschmidt lu ludcmp trisolv"
poly_medley="deriche floyd-warshall nussinov"
poly_stencils="adi fdtd-2d heat-3d jacobi-1d jacobi-2d seidel-2d"
#BENCHMARKS=$poly_datamining" "$poly_blas" "$poly_kernels" "$poly_solvers" "$poly_medley" "$poly_stencils
BENCHMARKS="$poly_kernels" 

##########################################################################
###---------------------------Check Paths----------------------------#####
if [ -d "$M5_PATH" ]; then
  echo 'gem5 found @ '$M5_PATH
else
  echo 'ERROR: gem5 not found @ '$M5_PATH' - check M5_PATH variable'
  exit 1;
fi

if [ -d "$benchmark_dir" ]; then
  echo 'Benchmarks found @ '$venchmark_dir
else
  echo 'ERROR: Benchmarks not found @ '$benchmark_dir' - check benchmark_dir variable'
  exit 1;
fi

if [ -f "$configs_init" ]; then
  echo 'Caches_init.py found @ '$configs_init
else
  echo 'ERROR: Caches_init.py not found @ '$configs_init' - check configs_init variable'
  exit 1;
fi

if [ -d "$results_path" ]; then
  echo 'Results Destination found @ '$results_path
else
  echo 'WARNING: Results Destination not found @ '$results_path
  mkdir "$results_path"
fi

cd "$M5_PATH"
mkdir "$results_dir"

##########################################################################
###--------------------Fault Free Case Simulation--------------------#####
if [ "$fcache_type" = "IL1" ]; then
    cp $configs_init $configs_inter
	sed -i '/^\s*il1_cache.*/a \ \ \ \ fta_predictor\ =\ True' $configs_inter
	sed -i "/^\s*il1_cache.*/a \ \ \ \ sub_blocks\ =\ $fta_sub_blocks" $configs_inter
	sed -i "/^\s*il1_cache.*/a \ \ \ \ sampling_predictor\ =\ $fta_sampling_rate" $configs_inter
	sed -i '/^\s*il1_cache.*/a \ \ \ \ faulty_cache\ =\ True' $configs_inter
	cp $configs_inter $configs_write
elif [ "$fcache_type" = "DL1" ]; then
    cp $configs_init $configs_inter
	sed -i '/^\s*dl1_cache.*/a \ \ \ \ fta_predictor\ =\ True' $configs_inter
	sed -i "/^\s*dl1_cache.*/a \ \ \ \ sub_blocks\ =\ $fta_sub_blocks" $configs_inter
	sed -i "/^\s*dl1_cache.*/a \ \ \ \ sampling_predictor\ =\ $fta_sampling_rate" $configs_inter
	sed -i '/^\s*dl1_cache.*/a \ \ \ \ faulty_cache\ =\ True' $configs_inter
	cp $configs_inter $configs_write
else
	echo 'ERROR: wrong faulty_cache: '$faulty_cache' @ faulty'
	exit 1;
fi
	         
cache_faulty_map=$fltmaps_dir"/"$fault_map_select"/"$pfail_select"/create-subitmap_sbdis"$fta_sub_blocks".c"

if [ -f "$cache_faulty_map" ]; then
  echo 'Faulty Map found @ '$cache_faulty_map
else
  echo 'ERROR: Faulty Map not found @ '$cache_faulty_map' - check cache_faulty_map variable @ faulty'
  exit 1;
fi

cp "$cache_faulty_map" "$cache_dir"create-subitmap.cc

#predictor scheme
rm $predictor_dir"fta_predictor.hh"
cp $predictor_dir"CG-SFP4/"$fta_pred_conf $predictor_dir"fta_predictor.hh"
	
#compile gem5
scons $build_dir -j$threads

cpu_config="--cpu-type=DerivO3CPU" 
mem_config="--caches --l1d_size="$l1d_size"kB --l1d_assoc="$l1d_assoc" --l1i_size="$l1i_size"kB --l1i_assoc="$l1i_assoc" --l2cache --l2_size="$l2_size" --l2_assoc="$l2_assoc" --mem-size="$mem_size
sim_config=$cpu_config" "$mem_config

running_jobs=0

for bmark in $BENCHMARKS
do

  if [ "$bmark" = "correlation" ]; then
    run_benchmark=$benchmark_dir"/datamining/correlation/correlation"
  elif [ "$bmark" = "covariance" ]; then
    run_benchmark=$benchmark_dir"/datamining/covariance/covariance"
  elif [ "$bmark" = "gemm" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/gemm/gemm"
  elif [ "$bmark" = "gemver" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/gemver/gemver"
  elif [ "$bmark" = "gesummv" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/gesummv/gesummv"
  elif [ "$bmark" = "symm" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/symm/symm"
  elif [ "$bmark" = "syr2k" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/syr2k/syr2k"
  elif [ "$bmark" = "syrk" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/syrk/syrk"
  elif [ "$bmark" = "trmm" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/blas/trmm/trmm"
  elif [ "$bmark" = "2mm" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/2mm/2mm"
  elif [ "$bmark" = "3mm" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/3mm/3mm"
  elif [ "$bmark" = "atax" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/atax/atax"
  elif [ "$bmark" = "bicg" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/bicg/bicg"
  elif [ "$bmark" = "doitgen" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/doitgen/doitgen"
  elif [ "$bmark" = "mvt" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/kernels/mvt/mvt"
  elif [ "$bmark" = "cholesky" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/cholesky/cholesky"
  elif [ "$bmark" = "durbin" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/durbin/durbin"
  elif [ "$bmark" = "gramschmidt" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/gramschmidt/gramschmidt"
  elif [ "$bmark" = "lu" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/lu/lu"
  elif [ "$bmark" = "ludcmp" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/ludcmp/ludcmp"
  elif [ "$bmark" = "trisolv" ]; then
    run_benchmark=$benchmark_dir"/linear-algebra/solvers/trisolv/trisolv"
  elif [ "$bmark" = "deriche" ]; then
    run_benchmark=$benchmark_dir"/medley/deriche/deriche"
  elif [ "$bmark" = "floyd-warshall" ]; then
    run_benchmark=$benchmark_dir"/medley/floyd-warshall/floyd-warshall"
  elif [ "$bmark" = "nussinov" ]; then
    run_benchmark=$benchmark_dir"/medley/nussinov/nussinov"
  elif [ "$bmark" = "adi" ]; then
    run_benchmark=$benchmark_dir"/stencils/adi/adi"
  elif [ "$bmark" = "fdtd-2d" ]; then
    run_benchmark=$benchmark_dir"/stencils/fdtd-2d/fdtd-2d"
  elif [ "$bmark" = "heat-3d" ]; then
    run_benchmark=$benchmark_dir"/stencils/heat-3d/heat-3d"
  elif [ "$bmark" = "jacobi-1d" ]; then
    run_benchmark=$benchmark_dir"/stencils/jacobi-1d/jacobi-1d"
  elif [ "$bmark" = "jacobi-2d" ]; then
    run_benchmark=$benchmark_dir"/stencils/jacobi-2d/jacobi-2d"
  elif [ "$bmark" = "seidel-2d" ]; then
    run_benchmark=$benchmark_dir"/stencils/seidel-2d/seidel-2d"
  else
    echo 'ERROR: wrong benchmark: '$bmark' @ clean'
    exit 1;
  fi

  echo $run_bmarks

  out_config="fta_cache_pfail_"$pfail_select"_subblk_"$fta_sub_blocks
  ./$build_dir --outdir=$results_dir"/"$bmark"_"$out_config --stats-file=$bmark"_"$out_config configs/example/se.py -c $run_benchmark $sim_config &
  running_jobs=$(($running_jobs+1))
  if [ $running_jobs -eq $threads ]; then
    running_jobs=0
    wait
  fi
done # BENCHMARKS

##########################################################################
###------------Clean Simulation and Compress the Results-------------#####
scons -c

echo "All done, check for the results @ "$results_dir
