fname = '../fundamental/matches.txt';
file = fopen(fname, 'r');
N = 500;
P = fscanf(file, '%f%f%f%f', [4,N]);
fclose(file);
X = ones(6, size(P, 2));
X(1 : 2, :) = P(1 : 2, :);
X(4 : 5, :) = P(3 : 4, :);
F = fundmatrix(X(:, 1:8));