using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SoftBody.Scripts
{
       public static class GraphClustering
    {
        public struct Cluster
        {
            public List<int> constraints;
            public HashSet<int> particles;
            public int colorGroup;
        }

        public static List<Cluster> CreateClusters(List<SoftBodySimulator.Constraint> constraints, int particleCount, int targetClustersPerParticle = 8)
        {
            // Build adjacency information
            var particleToConstraints = new List<int>[particleCount];
            for (var i = 0; i < particleCount; i++)
            {
                particleToConstraints[i] = new List<int>();
            }

            for (var i = 0; i < constraints.Count; i++)
            {
                particleToConstraints[constraints[i].particleA].Add(i);
                particleToConstraints[constraints[i].particleB].Add(i);
            }

            // Create initial clusters - one per constraint
            var clusters = new List<Cluster>();
            for (var i = 0; i < constraints.Count; i++)
            {
                var cluster = new Cluster
                {
                    constraints = new List<int> { i },
                    particles = new HashSet<int> { constraints[i].particleA, constraints[i].particleB },
                    colorGroup = -1
                };
                clusters.Add(cluster);
            }

            // Merge clusters to reduce total count
            var targetClusterCount = Mathf.Max(1, constraints.Count / targetClustersPerParticle);
            
            while (clusters.Count > targetClusterCount)
            {
                // Find best pair to merge
                int bestI = -1, bestJ = -1;
                var bestSharedParticles = 0;

                for (var i = 0; i < clusters.Count; i++)
                {
                    for (var j = i + 1; j < clusters.Count; j++)
                    {
                        var sharedParticles = clusters[i].particles.Intersect(clusters[j].particles).Count();
                        if (sharedParticles > bestSharedParticles)
                        {
                            bestSharedParticles = sharedParticles;
                            bestI = i;
                            bestJ = j;
                        }
                    }
                }

                if (bestI == -1) break; // No more merges possible

                // Merge clusters
                clusters[bestI].constraints.AddRange(clusters[bestJ].constraints);
                clusters[bestI].particles.UnionWith(clusters[bestJ].particles);
                clusters.RemoveAt(bestJ);
            }

            Debug.Log($"Created {clusters.Count} clusters from {constraints.Count} constraints");
            return clusters;
        }

        public static void ColorClusters(List<Cluster> clusters, List<SoftBodySimulator.Constraint> constraints)
        {
            // Build cluster adjacency
            var adjacency = new HashSet<int>[clusters.Count];
            for (var i = 0; i < clusters.Count; i++)
            {
                adjacency[i] = new HashSet<int>();
            }

            // Two clusters are adjacent if they share particles
            for (var i = 0; i < clusters.Count; i++)
            {
                for (var j = i + 1; j < clusters.Count; j++)
                {
                    if (clusters[i].particles.Intersect(clusters[j].particles).Any())
                    {
                        adjacency[i].Add(j);
                        adjacency[j].Add(i);
                    }
                }
            }

            // Greedy coloring of clusters
            var clusterList = clusters.ToList(); // Make a copy to modify
            var maxColor = 0;
            
            for (var i = 0; i < clusterList.Count; i++)
            {
                var usedColors = new HashSet<int>();
                foreach (var adj in adjacency[i])
                {
                    if (clusterList[adj].colorGroup >= 0)
                    {
                        usedColors.Add(clusterList[adj].colorGroup);
                    }
                }

                // Find first available color
                var color = 0;
                while (usedColors.Contains(color)) color++;
                
                var cluster = clusterList[i];
                cluster.colorGroup = color;
                clusterList[i] = cluster;
                
                maxColor = Mathf.Max(maxColor, color);

                // Apply color to all constraints in cluster
                foreach (var constraintIdx in cluster.constraints)
                {
                    var constraint = constraints[constraintIdx];
                    constraint.colorGroup = color;
                    constraints[constraintIdx] = constraint;
                }
            }

            Debug.Log($"Graph clustering complete: {maxColor + 1} color groups needed");
        }
    }
}