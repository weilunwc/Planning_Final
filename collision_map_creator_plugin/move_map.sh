dirlist=$(find $1 -mindepth 1 -maxdepth 1 -type d)
for dir in $dirlist
do
    (
        cd $dir
        DIRNAME="${PWD##*/}"
        pwd
    )
done
