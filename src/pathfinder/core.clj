(ns pathfinder.core
  (:require [clojure.math.numeric-tower :as math])
  (:require [clojure.set :as set])
  (:require [clojure.data.priority-map :refer [priority-map]])
  (:require [quil.core :as q])
  (:gen-class))

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
   :d [[14 19] [18 20] [21 17] [14 13]]
   :e [[18 10] [23 6] [19 3]]
   :f [[22 19] [28 19] [28 9] [22 9]]
   :g [[25 6] [28 8] [31 6] [31 2] [28 1] [25 2]]
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
       (map :to)))

(defn h [n]
  (distance (second n) (second goal)))

(defn g [current-travelled from to]
  (+ current-travelled (distance (second from) (second to))))

(defn A* []
  (loop [open-list (priority-map start (h start))
         closed-list #{}
         came-from {}
         g-score (assoc (zipmap (search-space) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (filter (comp not (partial contains? closed-list)) (get-visible-vertices current))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (get g-score current) current %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            f-score' (update-map #(+ %2 (h %1)) g-score')]
        (if (= goal current)
          (loop [current goal path []]
            (if (= current nil)
              (reverse path)
              (recur (get came-from current) (conj path current))))
          (recur
            (into (pop open-list) f-score')                 ;; add improved neighbors to open list
            (conj closed-list current)                      ;; add current vertex to the closed list
            (merge came-from (zipmap (keys g-score') (repeat current))) ;; update path to neighbor
            (merge-with min g-score g-score')))))))         ;; update the g-scores with best scores

(defn scale [x y]
  [(* x 20) (- (q/height) (* y 20))])

(defn draw []
  (let [[start-x start-y] (second start)
        [goal-x goal-y] (second goal)
        path (A*)]
    (q/with-translation
      [50 5]
      (q/background 255)
      (q/stroke 0 0 0)
      (q/stroke-weight 2)
      (doseq [shape (vals shapes)]                          ;; draw each obstacle
        (q/begin-shape)
        (doseq [[x y] shape]
          (apply q/vertex (scale x y)))
        (q/end-shape :close))
      (q/stroke 255 0 0)
      (q/stroke-weight 10)
      (apply q/point (scale start-x start-y))               ;; draw start point
      (apply q/point (scale goal-x goal-y))                 ;; draw goal point
      (q/stroke-weight 2)
      (q/no-fill)
      (q/begin-shape)                                       ;; draw A* path
      (doseq [[_ [x y]] path]
        (apply q/vertex (scale x y)))
      (q/end-shape))))

(defn -main []
  ;; print the A* path to the console
  (doall (map println (A*)))
  ;; draw the A* path using quil
  (q/sketch
    :title "Assignment 1"
    :size [800 400]
    :renderer :p2d
    :draw draw
    :features [:exit-on-close]))
