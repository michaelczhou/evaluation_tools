# KITTI odometry evaluation tool

## Usage

Each trajectory file `xx.txt` contains a N x 12 table, where N is the number of frames of the sequence. Row i represents the i'th pose via a 3x4 transformation matrix.
To evaluate one or more given trajectories, such as `09_pred.txt` and `10_pred.txt`  in the `data` folder:

        python evaluation.py --result_dir=./data/ --eva_seqs=09_pred,10_pred 

To evaluate all trajectories in the `data` folder:

        python evaluation.py --result_dir=./data/ --eva_seqs=* 

The evaluation results will be generated in folder `./data/xx_eval/`. 

## Reference
<a href="http://www.cvlibs.net/datasets/kitti/eval_odometry.php" target="_blank">KITTI odometry development kit</a>
