python3 -m cProfile -o scripts/output.pstats envs/cheetah_mpc_env.py --render
gprof2dot --colour-nodes-by-selftime -f pstats scripts/output.pstats | dot -Tpng -o scripts/output.png 
