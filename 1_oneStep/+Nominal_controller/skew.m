function skewSymmetric = skew(vector)

skewSymmetric = zeros(3,3);
skewSymmetric(2,3) = -vector(1);
skewSymmetric(1,3) = vector(2);
skewSymmetric(1,2) = vector(3);

skewSymmetric = skewSymmetric - skewSymmetric.';

end