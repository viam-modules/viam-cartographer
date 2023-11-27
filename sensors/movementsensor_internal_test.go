package sensors

import (
	"testing"
	"time"

	"go.viam.com/test"
)

func TestAverageReadingTimes(t *testing.T) {
	t.Run("a equals b", func(t *testing.T) {
		a := time.Now()
		b := a
		test.That(t, a, test.ShouldEqual, averageReadingTimes(a, b))
	})

	t.Run("b 10 msec larger than a", func(t *testing.T) {
		a := time.Now()
		b := a.Add(10 * time.Millisecond)
		test.That(t, averageReadingTimes(a, b), test.ShouldEqual, a.Add(5*time.Millisecond))
	})

	t.Run("a 66 msec larger than b", func(t *testing.T) {
		b := time.Now()
		a := b.Add(66 * time.Millisecond)
		test.That(t, averageReadingTimes(a, b), test.ShouldEqual, b.Add(33*time.Millisecond))
	})
}
