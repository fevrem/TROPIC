function v = CAT_ALGEBRAIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 132);
  end
  v = vInitialized;
end
