function z = sat(sigma, y)

for k=1:length(y)
    if y > sigma
        z = sigma;
    elseif y < -sigma
        z = -sigma;
    else
        z = y;
    end
end

end