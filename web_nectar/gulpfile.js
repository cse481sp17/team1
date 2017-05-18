var gulp = require('gulp'),
    clean = require('gulp-clean'),
    sass = require('gulp-sass'),
    autoprefixer = require('gulp-autoprefixer'),
    minifycss = require('gulp-minify-css'),
    jade = require('gulp-jade'),
    browserSync = require('browser-sync').create();

// Build
gulp.task('build', ['clean'], function() {
  gulp.start('templates', 'styles', 'bower');
});

// Compile html
gulp.task('templates', function() {
  return gulp.src('jade/*.jade')
    .pipe(jade())
    .pipe(gulp.dest('./'))
    .pipe(browserSync.stream())
});

// Compile css
gulp.task('styles', function() {
  return gulp.src('scss/*.scss')
    .pipe(sass())
    .pipe(autoprefixer('last 2 version'))
    .pipe(minifycss())
    .pipe(gulp.dest('./'))
    .pipe(browserSync.stream())
});

gulp.task('bower', function() {
    return gulp.src(['./bower_components/**/dist/**',
        './bower_components/**/css/**',
        './bower_components/**/fonts/**'])
      .pipe(gulp.dest('dist'));
});

// Clean
gulp.task('clean', function () {
  return gulp.src(['*.html', '*.css', 'dist/'])
    .pipe(clean());
});

// Static Server + watching scss/jade files
gulp.task('serve', ['build'], function() {
  browserSync.init({
      server: "./"
  });
  gulp.watch('scss/*', ['styles']);
  gulp.watch('jade/*', ['templates']);
  gulp.watch('js/*').on('change', browserSync.reload);
});
