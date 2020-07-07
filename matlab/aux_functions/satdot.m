function z=satdot(sigma, y, ydot)

for k = 1:length(y)
    if y > sigma
        z = 0;
    elseif y < -sigma
        z = 0;
    else
        z = ydot;
    end
end

end