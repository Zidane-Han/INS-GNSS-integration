% genDltaQ
function DeltaQ = genDeltaQ(wnbbsins)

       DeltaQ(:,:) = [0 -wnbbsins(1,:) -wnbbsins(2,:) -wnbbsins(3,:);
                      wnbbsins(1,:) 0 wnbbsins(3,:) -wnbbsins(2,:);
                      wnbbsins(2,:) -wnbbsins(3,:) 0 wnbbsins(1,:);
                      wnbbsins(3,:) wnbbsins(2,:) -wnbbsins(1,:) 0];

