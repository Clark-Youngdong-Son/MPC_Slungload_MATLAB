function vector = skew_inverse(skewSymmetric)

vector = zeros(3,1);
vector(1) = -skewSymmetric(2,3);
vector(2) = skewSymmetric(1,3);
vector(3) = -skewSymmetric(1,2);

end