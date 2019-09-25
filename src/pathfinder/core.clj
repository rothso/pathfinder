(ns pathfinder.core
  (:require [clojure.math.numeric-tower :as math]))

(defn rotate [n a]
  (let [l (count a)
        off (mod (+ (mod n l) l) l)]
    (concat (drop off a) (take off a))))

(def shapes
  {:a [[2 6] [17 6] [17 1] [2 1]]
   :b [[0 14] [6 19] [9 15] [7 8] [1 9]]
   :c [[10 8] [12 15] [14 8]]
   :d [[14 19] [18 20] [20 17] [14 13]]
   :e [[18 10] [23 6] [19 3]]
   :f [[22 19] [28 19] [28 9] [22 9]]
   :g [[25 6] [29 8] [31 6] [31 2] [28 1] [25 2]]
   :h [[29 17] [31 19] [34 16] [32 8]]})

(defn distance [vert-a vert-b]
  (math/sqrt (reduce + (map (fn [x] (math/expt x 2)) (map - vert-a vert-b)))))

(defn create-edge [vert-a vert-b]
  {:from vert-a :to vert-b :dist (distance vert-a vert-b)})

(defn neighboring-edges [verts]
  (concat
    (map create-edge verts (rotate 1 verts))
    (map create-edge verts (rotate -1 verts))))

(defn build-edges [shapes]
  (flatten (map neighboring-edges (vals shapes))))

(defn ccw [[ax ay] [bx by] [cx cy]]
  (> (* (- cy ay) (- bx ax))
     (* (- by ay) (- cx ax))))

(defn intersect [a b c d]
  (and (not= (ccw a c d) (ccw b c d))
       (not= (ccw a b c) (ccw a b d))))

(def start [1 3])
(def goal [34 19])