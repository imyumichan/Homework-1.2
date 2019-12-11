function dists = compute_distances(points_2d,true_2d)

      f = [points_2d(1,:)'; points_2d(2,:)'];


      m = [true_2d(1,:)'; true_2d(2,:)'];
      
      dists = abs(f-m);



end