using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace SpatialPrintSort
{
    public class Graph
    {
        public List<Point3d> Nodes { get; private set; }
        public List<Tuple<Point3d, Point3d>> Edges { get; private set; }

        public Graph()
        {
            Nodes = new List<Point3d>();
            Edges = new List<Tuple<Point3d, Point3d>>();
        }

        public void AddNode(Point3d node)
        {
            Nodes.Add(node);
        }

        public void AddEdge(Point3d node1, Point3d node2)
        {
            Edges.Add(new Tuple<Point3d, Point3d>(node1, node2));
        }
        public List<List<Point3d>> FindConnectedComponents()
        {
            List<List<Point3d>> connectedComponents = new List<List<Point3d>>();
            HashSet<Point3d> visitedNodes = new HashSet<Point3d>();

            foreach (Point3d node in Nodes)
            {
                if (!visitedNodes.Contains(node))
                {
                    List<Point3d> connectedComponent = BreadthFirstSearch(node);
                    connectedComponents.Add(connectedComponent);
                    visitedNodes.UnionWith(connectedComponent);
                }
            }

            return connectedComponents;
        }

        private List<Point3d> BreadthFirstSearch(Point3d startNode)
        {
            List<Point3d> connectedComponent = new List<Point3d>();
            Queue<Point3d> queue = new Queue<Point3d>();
            HashSet<Point3d> visitedNodes = new HashSet<Point3d>();

            queue.Enqueue(startNode);
            visitedNodes.Add(startNode);

            while (queue.Count > 0)
            {
                Point3d currentNode = queue.Dequeue();
                connectedComponent.Add(currentNode);

                List<Point3d> neighbors = GetNeighbors(currentNode);
                foreach (Point3d neighbor in neighbors)
                {
                    if (!visitedNodes.Contains(neighbor))
                    {
                        queue.Enqueue(neighbor);
                        visitedNodes.Add(neighbor);
                    }
                }
            }

            return connectedComponent;
        }
        private List<Point3d> GetNeighbors(Point3d node)
        {
            List<Point3d> neighbors = new List<Point3d>();
            foreach (var edge in Edges)
            {
                if (edge.Item1 == node)
                    neighbors.Add(edge.Item2);
                else if (edge.Item2 == node)
                    neighbors.Add(edge.Item1);
            }
            return neighbors;
        }
        public Polyline FindSolution()
        {
            List<List<Point3d>> connectedComponents = FindConnectedComponents();

            if (connectedComponents.Count == 0)
                return null;

            // Find the longest path by traversing the edges
            List<Point3d> longestPath = new List<Point3d>();

            // Keep track of visited edges
            HashSet<Tuple<Point3d, Point3d>> visitedEdges = new HashSet<Tuple<Point3d, Point3d>>();

            // Start the path with the first edge in the graph
            Tuple<Point3d, Point3d> startEdge = Edges.First();
            visitedEdges.Add(startEdge);
            longestPath.Add(startEdge.Item1);
            longestPath.Add(startEdge.Item2);

            // Traverse the graph to find the longest path
            TraverseGraph(startEdge.Item2, longestPath, visitedEdges);

            // Create a polyline from the longest path
            Polyline polyline = new Polyline(longestPath);

            return polyline;
        }

        private void TraverseGraph(Point3d currentNode, List<Point3d> path, HashSet<Tuple<Point3d, Point3d>> visitedEdges)
        {
            // Find the neighbors of the current node
            List<Point3d> neighbors = GetNeighbors(currentNode);

            // Find the unvisited edges connected to the current node
            List<Tuple<Point3d, Point3d>> unvisitedEdges = Edges.Where(e =>
                (e.Item1 == currentNode && !visitedEdges.Contains(e)) ||
                (e.Item2 == currentNode && !visitedEdges.Contains(e))
            ).ToList();

            // Sort the unvisited edges by their length
            //unvisitedEdges.Sort((a, b) =>
            //    a.Item1.DistanceTo(a.Item2).CompareTo(b.Item1.DistanceTo(b.Item2))
            //);

            // Sort the unvisited edges based on their weighted parameters
            unvisitedEdges.Sort((a, b) =>
                GetEdgeWeight(a).CompareTo(GetEdgeWeight(b))
            );

            // Traverse each unvisited edge
            foreach (var edge in unvisitedEdges)
            {
                visitedEdges.Add(edge);

                // Determine the next node in the path
                Point3d nextNode = edge.Item1 == currentNode ? edge.Item2 : edge.Item1;

                // Add the next node to the path
                path.Add(nextNode);

                // Recursively traverse the next node
                TraverseGraph(nextNode, path, visitedEdges);

                // Break the loop if all edges have been visited
                if (visitedEdges.Count == Edges.Count)
                    RhinoApp.WriteLine("All edges traversed");
                    break;
            }
        }
        private double GetEdgeWeight(Tuple<Point3d, Point3d> edge)
        {
            // Calculate the weight based on edge properties or custom logic
            // Return a double value representing the weight
            // You can access edge.Item1 and edge.Item2 to get the start and end points of the edge
            // Retrieve the start and end points of the edge
            Point3d startPt = edge.Item1;
            Point3d endPt = edge.Item2;

            // Calculate the three weights based on different criteria
            double weight1 = startPt.DistanceTo(endPt); // Distance between start and end points
            double weight2 = Math.Abs(startPt.Z - endPt.Z); // Absolute difference in Z coordinates
            double weight3 = Math.Max(startPt.Y, endPt.Y); // Maximum Y coordinate

            // Calculate the overall weight as a combination of the three weights
            double overallWeight = weight1 + weight2 + weight3;

            return overallWeight;
        }
    }
    public class GraphCreationCommand : Command
    {
        public override string EnglishName => "GraphCreation";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // Select curves in Rhino
            var go = new GetObject();
            go.SetCommandPrompt("Select curves to create graph");
            go.GeometryFilter = Rhino.DocObjects.ObjectType.Curve;
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            // Initialize a new graph
            var graph = new Graph();

            // Iterate over each selected curve
            foreach (var objRef in go.Objects())
            {
                var curve = objRef.Curve();
                if (curve != null)
                {
                    // Get the start and end points of the curve
                    var startPt = curve.PointAtStart;
                    var endPt = curve.PointAtEnd;

                    // Add the start and end points as nodes to the graph
                    graph.AddNode(startPt);
                    graph.AddNode(endPt);

                    // Add an edge between the start and end points
                    graph.AddEdge(startPt, endPt);
                }
            }

            // Print some information about the graph
            RhinoApp.WriteLine("Number of nodes: " + graph.Nodes.Count);
            RhinoApp.WriteLine("Number of edges: " + graph.Edges.Count);

            // Find connected components
            List<List<Point3d>> connectedComponents = graph.FindConnectedComponents();
            RhinoApp.WriteLine("Number of connected components: " + connectedComponents.Count);
            foreach (var component in connectedComponents)
            {
                RhinoApp.WriteLine("Component size: " + component.Count);
            }
            // Create copies of the edges for each connected component
            List<Color> componentColors = GenerateColors(connectedComponents.Count);

            for (int i = 0; i < connectedComponents.Count; i++)
            {
                var component = connectedComponents[i];
                var color = componentColors[i];
                

                foreach (var edge in graph.Edges)
                {
                    if (component.Contains(edge.Item1) && component.Contains(edge.Item2))
                    {
                        var line = new Line(edge.Item1, edge.Item2);
                        
                        doc.Objects.AddLine(line, new ObjectAttributes { ColorSource = ObjectColorSource.ColorFromObject, ObjectColor = color });
                       
                    }
                }

            }
            

            // Find a solution that connects all the edges into a single polyline
            Polyline solution = graph.FindSolution();
            if (solution != null)
            {

                doc.Objects.AddPolyline(solution);
                
            }
            


            // Redraw the viewport
            doc.Views.Redraw();
            return Result.Success;
        }
        private List<Color> GenerateColors(int count)
        {
            var colors = new List<Color>();
            var random = new Random();

            for (int i = 0; i < count; i++)
            {
                byte r = (byte)random.Next(256);
                byte g = (byte)random.Next(256);
                byte b = (byte)random.Next(256);
                colors.Add(Color.FromArgb(r, g, b));
            }

            return colors;
        }
    }
}
