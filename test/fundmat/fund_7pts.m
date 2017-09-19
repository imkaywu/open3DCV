addpath(genpath('/Users/BlacKay/Documents/Projects/Open Source/VGG_MVG'));
matchesfile='../fundamental/matches.txt';

% read in matches
[x1, y1, x2, y2]=textread(matchesfile, '%f%f%f%f', 'commentstyle', 'shell');
pts0(:, :)=[x1'; y1']';
pts1(:, :)=[x2'; y2']';

x1 = [pts0(1:7,:), ones(7, 1)]';
x2 = [pts1(1:7,:), ones(7, 1)]';

F=vgg_F_from_7pts_2img(x1, x2);