(ns pathfinder.core
  (:require [clojure.math.numeric-tower :as math])
  (:require [clojure.set :as set])
  (:require [clojure.data.priority-map :refer [priority-map, priority-map-by]])
  (:require [quil.core :as q])
  (:gen-class))

(defn rotate [n a]
  (let [l (count a)
        off (mod (+ (mod n l) l) l)]
    (concat (drop off a) (take off a))))

(defn update-map [f m]
  (reduce-kv (fn [m k v]
               (assoc m k (f k v))) {} m))

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

(defn shape-verts [shapes]
  (reduce-kv (fn [m k v] (conj m (map (fn [x] [k x]) v))) [] shapes))

(defn obstacle-edges [shapes]
  (set (flatten (map neighboring-edges (shape-verts shapes)))))

(defn search-space [shapes goal]
  (conj (apply concat (shape-verts shapes)) goal))

(defn visible? [obstacle-edges {[_ a] :from [_ b] :to}]
  (not-any? (fn [{[_ c] :from [_ d] :to}] (intersecting? a b c d)) obstacle-edges))

(defn valid-edge? [obstacle-edges {[from] :from [to] :to :as edge}]
  (or (not= from to)                                        ;; going to different shape
      (contains? obstacle-edges edge)                       ;; adjacent on the same shape
      (contains? obstacle-edges (flip edge))))              ;; adjacent on the same shape

(defn get-visible-vertices [shapes goal vertex]
  (let [obstacle-edges (obstacle-edges shapes)]
    (->> (map #(create-edge vertex %) (search-space shapes goal))
         (filter (partial valid-edge? obstacle-edges))      ;; not going through the shape
         (filter (partial visible? obstacle-edges))         ;; not going through other shapes
         (map :to))))

(defn h [n goal]
  (distance (second n) (second goal)))

(defn g [current-travelled from to]
  (+ current-travelled (distance (second from) (second to))))

(defn A* [shapes start goal]
  (loop [open-list (priority-map start (h start goal))
         closed-list #{}
         came-from {}
         g-score (assoc (zipmap (search-space shapes goal) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (->> current                          ;; get all unvisited neighbors
                           (get-visible-vertices shapes goal)
                           (filter (comp not (partial contains? closed-list))))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (get g-score current) current %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            f-score' (update-map #(+ %2 (h %1 goal)) g-score')]
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

(defn u [c g-score h-score]
  (if (= h-score 0) ##Inf (/ (- c g-score) h-score)))

(defn potential-search [shapes start goal cost]
  (loop [open-list (priority-map-by > start (u cost 0 (h start goal)))
         closed-list #{}
         came-from {}
         g-score (assoc (zipmap (search-space shapes goal) (repeat ##Inf)) start 0)]
    (when (not (empty? open-list))
      (let [current (key (peek open-list))
            neighbors (->> current                          ;; get all unvisited neighbors
                           (get-visible-vertices shapes goal)
                           (filter (comp not (partial contains? closed-list))))
            g-score' (->> neighbors                         ;; get improved g-scores of neighbors
                          (map #(g (get g-score current) current %))
                          (zipmap neighbors)
                          (filter #(< (val %) (g-score (key %))))
                          (into {}))
            u-score' (update-map #(u cost %2 (h %1 goal)) g-score')]
        (if (>= (val (peek open-list)) 0)                   ;; u(n) < 0, no path found, return nil
          (if (= goal current)
            (loop [current goal path []]
              (if (= current nil)
                (reverse path)
                (recur (get came-from current) (conj path current))))
            (recur
              (into (pop open-list) u-score')               ;; add improved neighbors to open list
              (conj closed-list current)                    ;; add current vertex to the closed list
              (merge came-from (zipmap (keys g-score') (repeat current))) ;; update path to neighbor
              (merge-with min g-score g-score'))))))))      ;; update the g-scores with best scores

(defn scale [x y]
  [(* x 20) (- (q/height) (* y 20))])

(defn draw [shapes start goal path]
  (let [[start-x start-y] (second start)
        [goal-x goal-y] (second goal)]
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
      (q/begin-shape)                                       ;; draw search path
      (doseq [[_ [x y]] path]
        (apply q/vertex (scale x y)))
      (q/end-shape))))

(defn prompt-read [prompt]
  (print (format "%s: " prompt))
  (flush)
  (read-line))

(defn -main []
  (let [shapes {:a [[2 6] [17 6] [17 1] [2 1]]
                :b [[0 14] [6 19] [9 15] [7 8] [1 9]]
                :c [[10 8] [12 15] [14 8]]
                :d [[14 19] [18 20] [21 17] [14 13]]
                :e [[18 10] [23 6] [19 3]]
                :f [[22 19] [28 19] [28 9] [22 9]]
                :g [[25 6] [28 8] [31 6] [31 2] [28 1] [25 2]]
                :h [[29 17] [31 19] [34 16] [32 8]]}
        start [:start [1 3]]
        goal [:goal [34 19]]
        cost (Integer/parseInt (prompt-read "Enter the cost (C)"))
        path (potential-search shapes start goal cost)]
    (if (nil? path)
      (println "No path found")
      (do
        ;; print the search path to the console
        (doall (map println path))
        ;; draw the search path using quil
        (q/sketch
          :title "Assignment 2"
          :size [800 400]
          :renderer :p2d
          :draw #(draw shapes start goal path)
          :features [:exit-on-close])))))
