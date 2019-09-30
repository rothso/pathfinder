(defproject pathfinder "1.0.0"
  :description "A* pathfinding project."
  :url "https://github.com/rothso/pathfinder"
  :license {:name "The MIT License"
            :url "http://opensource.org/licenses/MIT"}
  :dependencies [[org.clojure/clojure "1.10.0"]
                 [org.clojure/math.numeric-tower "0.0.4"]
                 [org.clojure/math.combinatorics "0.1.6"]
                 [org.clojure/data.priority-map "0.0.10"]
                 [quil "3.0.0"]]
  :aot [pathfinder.core]
  :main pathfinder.core
  :repl-options {:init-ns pathfinder.core})
