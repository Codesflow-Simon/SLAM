void rangeTest() {
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  Values values;

  graph -> add(PriorFactor<Vector3>((Key)0, Vector3(1,0,0)));
  graph -> add(PriorFactor<Vector3>((Key)1, Vector3(0,1,0)));
  graph -> add(PriorFactor<Vector3>((Key)2, Vector3(0,0,1)));
  graph -> add(PriorFactor<Vector3>((Key)3, Vector3(0,0,0)));
  values.insert((Key)0, Vector3(1,0,0));
  values.insert((Key)1, Vector3(0,1,0));
  values.insert((Key)2, Vector3(0,0,1));
  values.insert((Key)3, Vector3(0,0,0));
  values.insert((Key)4, Vector3(-10,10,-10));

  graph -> add(RangeFactor<Vector3, Vector3, double>((Key)0, (Key)(4), 1.0, distance_noise_model));
  graph -> add(RangeFactor<Vector3, Vector3, double>((Key)1, (Key)(4), 1.0, distance_noise_model));
  graph -> add(RangeFactor<Vector3, Vector3, double>((Key)2, (Key)(4), sqrt(3), distance_noise_model));
  graph -> add(RangeFactor<Vector3, Vector3, double>((Key)3, (Key)(4), sqrt(2), distance_noise_model));

  LevenbergMarquardtOptimizer optimizer(*graph, values);
  values = optimizer.optimize(); 
  cout << "test run " << values.at<Vector3>((Key) 4) << endl << endl;
}