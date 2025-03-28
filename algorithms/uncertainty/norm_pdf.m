function y = norm_pdf(x, sigma, mu)
    y = (1/(sqrt(2*pi)*sigma))*exp(-0.5*((x-mu)/sigma).^2);
end

