e=0;
for i in $(cat jockeys);do 
f=$(printf %04i $e);e=$(($e+1));
convert -size 300x48 canvas:black -fill green -font Bookman-DemiItalic -pointsize 24 -gravity Center -type truecolor -depth 8 -draw "text 1,1 $i" tmp.bmp;
convert tmp.bmp +matte images/A$f.bmp
convert -size 300x48 canvas:black -fill 'gray(10%)' -font Bookman-DemiItalic -pointsize 24 -gravity Center -type truecolor -depth 8  -draw "text 1,1 $i" tmp.bmp;
convert tmp.bmp +matte images/I$f.bmp
done
rm tmp.bmp


