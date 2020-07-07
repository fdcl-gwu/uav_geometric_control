function [x, v, R, W, ei, eI] = split_to_states(X)

x = X(1:3);
v = X(4:6);
W = X(7:9);
R = reshape(X(10:18), 3, 3);
ei = X(19:21);
eI = X(22:24);

end