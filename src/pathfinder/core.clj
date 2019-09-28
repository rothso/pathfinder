(ns pathfinder.core
  (:require [clojure.math.numeric-tower :as math])
  (:require [clojure.set :as set])
  (:require [clojure.data.priority-map :refer [priority-map]]))

(defn rotate [n a]
  (let [l (count a)
        off (mod (+ (mod n l) l) l)]
    (concat (drop off a) (take off a))))

(defn update-map [f m]
  (reduce-kv (fn [m k v]
               (assoc m k (f k v))) {} m))

(def shapes
  {:a [[2 6] [17 6] [17 1] [2 1]]
   :b [[0 14] [6 19] [9 15] [7 8] [1 9]]
   :c [[10 8] [12 15] [14 8]]
   :d [[14 19] [18 20] [20 17] [14 13]]
   :e [[18 10] [23 6] [19 3]]
   :f [[22 19] [28 19] [28 9] [22 9]]
   :g [[25 6] [29 8] [31 6] [31 2] [28 1] [25 2]]
   :h [[29 17] [31 19] [34 16] [32 8]]})

(def start [:start [1 3]])
(def goal [:goal [34 19]])

(defn shape-verts []
  (reduce-kv (fn [m k v] (conj m (map (fn [x] [k x]) v))) [] shapes))

(defn distance [vert-a vert-b]
  (math/sqrt (reduce + (map #(math/expt % 2) (map - vert-a vert-b)))))

(defn create-edge [a b]
  {:from a :to b})

(defn flip [edge]
  (set/rename-keys edge {:from :to, :to :from}))

(defn neighboring-edges [verts]
  (map create-edge verts (rotate 1 verts)))

(defn ccw [[ax ay] [bx by] [cx cy]]
  (> (* (- cy ay) (- bx ax))
     (* (- by ay) (- cx ax))))

(defn intersecting? [a b c d]
  (and (not= (ccw a c d) (ccw b c d))
       (not= (ccw a b c) (ccw a b d))
       (and (distinct? a b c d))))

(defn obstacle-edges []
  (set (flatten (map neighboring-edges (shape-verts)))))

(defn search-space []
  (conj (apply concat (shape-verts)) goal))

(defn potential-edges [vertex]
  (map #(create-edge vertex %) (search-space)))

(defn visible? [{[_ a] :from [_ b] :to}]
  (not-any? (fn [{[_ c] :from [_ d] :to}] (intersecting? a b c d)) (obstacle-edges)))

(defn valid-edge? [{[from] :from [to] :to :as edge}]
  (or (not= from to)
      (contains? (obstacle-edges) edge)
      (contains? (obstacle-edges) (flip edge))))

(defn get-visible-vertices [vertex]
  (->> (potential-edges vertex)
       (filter valid-edge?)
       (filter visible?)
       (map #(:to %))))

(defn h [n]
  (distance (second n) (second goal)))

(defn A* []
  (loop [open-list (priority-map start (h start))
         closed-list #{}
         came-from []
         g-score (assoc (zipmap (search-space) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [g (fn [from to] (+ (get g-score from) (distance (second from) (second to))))
            current (key (peek open-list))
            neighbors (filter #(not (contains? closed-list (:to %))) (get-visible-vertices current))
            g-score' (->> neighbors
                          (map #(g current %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            f-score' (update-map #(+ %2 (h %1)) g-score')]
        (println current)
        (if (= goal current)
          "hello"                                           ;; todo
          (recur
            (into (pop open-list) f-score')                 ;; add improved neighbors to open list
            (conj closed-list current)                      ;; add current vertex to the closed list
            came-from
            (merge-with min g-score g-score')))))))         ;; update the g-scores with the minimum