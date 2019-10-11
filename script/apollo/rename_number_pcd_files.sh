# For more than 10000 datas
#rename 's/^/0000/' [0-9].pcd
#rename 's/^/000/' [0-9][0-9].pcd
#rename 's/^/00/' [0-9][0-9][0-9].pcd
#rename 's/^/0/' [0-9][0-9][0-9][0-9].pcd

# For more than 1000 datas
rename 's/^/000/' [0-9].pcd
rename 's/^/00/' [0-9][0-9].pcd
rename 's/^/0/' [0-9][0-9][0-9].pcd
