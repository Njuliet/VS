% 
%   Copyright (C) 2016  Starsky Wong <sununs11@gmail.com>
% 
%   Note: The SIFT algorithm is patented in the United States and cannot be
%   used in commercial products without a license from the University of
%   British Columbia.  For more information, refer to the file LICENSE
%   that accompanied this distribution.

clear
tic
img1 = imread('Left.jpg');
img2 = imread('fullLeft.jpg');
img3 = imread('validLeft.jpg');
img4 = imread('Right.jpg');
[des1,loc1] = getFeatures(img1);
[des2,loc2] = getFeatures(img2);
[des3,loc3] = getFeatures(img3);
[des4,loc4] = getFeatures(img4);
matched = match(des1,des4);
drawFeatures(img1,loc1);
drawFeatures(img2,loc2);
drawFeatures(img3,loc3);
drawMatched(matched,img1,img4,loc1,loc4);
toc