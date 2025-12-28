import React, { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { gsap } from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';

import styles from './index.module.css';

// Register GSAP plugins
if (typeof window !== 'undefined') {
  gsap.registerPlugin(ScrollTrigger);
}

// Robot slider images - using placeholder SVGs for robots
const sliderImages = [
  {
    id: 1,
    title: 'Humanoid Robotics',
    subtitle: 'Building the Future of Human-Robot Interaction',
    gradient: 'linear-gradient(135deg, #0d47a1 0%, #1976d2 50%, #42a5f5 100%)',
  },
  {
    id: 2,
    title: 'AI-Powered Learning',
    subtitle: 'Revolutionizing Education with Artificial Intelligence',
    gradient: 'linear-gradient(135deg, #1565c0 0%, #1e88e5 50%, #64b5f6 100%)',
  },
  {
    id: 3,
    title: 'Physical AI Systems',
    subtitle: 'Bridging Theory and Real-World Applications',
    gradient: 'linear-gradient(135deg, #0d47a1 0%, #1565c0 50%, #1976d2 100%)',
  },
];

// Book modules with their icons and descriptions
const bookModules = [
  {
    title: 'Scope & Boundaries',
    description: 'Define the scope and boundaries of Physical AI & Humanoid Robotics in education contexts.',
    icon: '🎯',
    link: '/docs/01-scope-boundaries',
    color: '#2196f3',
  },
  {
    title: 'Ethical Dilemmas',
    description: 'Explore ethical considerations, dilemmas, and frameworks for responsible AI development.',
    icon: '⚖️',
    link: '/docs/02-ethical-dilemmas',
    color: '#9c27b0',
  },
  {
    title: 'Technical Concepts',
    description: 'Master core technical concepts including sensors, actuators, and control systems.',
    icon: '⚙️',
    link: '/docs/03-technical-concepts',
    color: '#f44336',
  },
  {
    title: 'Pedagogical Approaches',
    description: 'Learn effective teaching methodologies for robotics and AI education.',
    icon: '📚',
    link: '/docs/04-pedagogical-approaches',
    color: '#4caf50',
  },
  {
    title: 'Education Levels',
    description: 'Tailored content for different education levels from K-12 to higher education.',
    icon: '🎓',
    link: '/docs/05-education-levels',
    color: '#ff9800',
  },
  {
    title: 'Implementation Guide',
    description: 'Step-by-step guidance for implementing robotics programs in educational settings.',
    icon: '🛠️',
    link: '/docs/06-implementation-guidance',
    color: '#00bcd4',
  },
  {
    title: 'Privacy & Security',
    description: 'Best practices for ensuring privacy and security in AI-enabled educational systems.',
    icon: '🔒',
    link: '/docs/07-privacy-security',
    color: '#607d8b',
  },
  {
    title: 'Edge Kit',
    description: 'Hands-on guide to the Physical AI Edge Kit for practical robotics experimentation.',
    icon: '🤖',
    link: '/docs/edge-kit',
    color: '#e91e63',
  },
];

// Robot SVG Component
function RobotSVG({ className }: { className?: string }) {
  return (
    <svg className={className} viewBox="0 0 200 240" fill="none" xmlns="http://www.w3.org/2000/svg">
      {/* Robot Head */}
      <rect x="40" y="20" width="120" height="90" rx="20" fill="white" fillOpacity="0.15" stroke="white" strokeWidth="3"/>
      {/* Antenna */}
      <line x1="100" y1="20" x2="100" y2="0" stroke="white" strokeWidth="4" strokeLinecap="round"/>
      <circle cx="100" cy="0" r="8" fill="#4ade80" className={styles.antennaGlow}/>
      {/* Eyes */}
      <circle cx="70" cy="55" r="15" fill="white" fillOpacity="0.9"/>
      <circle cx="130" cy="55" r="15" fill="white" fillOpacity="0.9"/>
      <circle cx="70" cy="55" r="8" fill="#0d47a1"/>
      <circle cx="130" cy="55" r="8" fill="#0d47a1"/>
      <circle cx="73" cy="52" r="3" fill="white"/>
      <circle cx="133" cy="52" r="3" fill="white"/>
      {/* Mouth */}
      <rect x="60" y="80" width="80" height="10" rx="5" fill="#4ade80" className={styles.mouthGlow}/>
      {/* Neck */}
      <rect x="85" y="110" width="30" height="20" fill="white" fillOpacity="0.2"/>
      {/* Body */}
      <rect x="30" y="130" width="140" height="80" rx="15" fill="white" fillOpacity="0.15" stroke="white" strokeWidth="3"/>
      {/* Chest display */}
      <rect x="60" y="145" width="80" height="50" rx="8" fill="white" fillOpacity="0.1" stroke="white" strokeWidth="2"/>
      <circle cx="100" cy="170" r="15" fill="#4ade80" fillOpacity="0.5" className={styles.chestGlow}/>
      {/* Arms */}
      <rect x="5" y="135" width="25" height="60" rx="10" fill="white" fillOpacity="0.15" stroke="white" strokeWidth="2"/>
      <rect x="170" y="135" width="25" height="60" rx="10" fill="white" fillOpacity="0.15" stroke="white" strokeWidth="2"/>
      {/* Hands */}
      <circle cx="17" cy="205" r="12" fill="white" fillOpacity="0.2" stroke="white" strokeWidth="2"/>
      <circle cx="183" cy="205" r="12" fill="white" fillOpacity="0.2" stroke="white" strokeWidth="2"/>
    </svg>
  );
}

function HeroSlider() {
  const [currentSlide, setCurrentSlide] = useState(0);
  const sliderRef = useRef<HTMLDivElement>(null);
  const titleRef = useRef<HTMLHeadingElement>(null);
  const subtitleRef = useRef<HTMLParagraphElement>(null);
  const robotRef = useRef<HTMLDivElement>(null);

  // Auto-advance slider
  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentSlide((prev) => (prev + 1) % sliderImages.length);
    }, 5000);
    return () => clearInterval(interval);
  }, []);

  // Animate on slide change
  useEffect(() => {
    if (titleRef.current && subtitleRef.current && robotRef.current) {
      const tl = gsap.timeline();
      tl.fromTo(
        titleRef.current,
        { opacity: 0, y: 30 },
        { opacity: 1, y: 0, duration: 0.6, ease: 'power3.out' }
      )
        .fromTo(
          subtitleRef.current,
          { opacity: 0, y: 20 },
          { opacity: 1, y: 0, duration: 0.5, ease: 'power3.out' },
          '-=0.3'
        )
        .fromTo(
          robotRef.current,
          { opacity: 0, scale: 0.8, rotate: -10 },
          { opacity: 1, scale: 1, rotate: 0, duration: 0.7, ease: 'back.out(1.7)' },
          '-=0.4'
        );
    }
  }, [currentSlide]);

  // Initial animation
  useEffect(() => {
    if (typeof window !== 'undefined' && sliderRef.current) {
      gsap.fromTo(
        sliderRef.current,
        { opacity: 0 },
        { opacity: 1, duration: 1, ease: 'power2.out' }
      );
    }
  }, []);

  const currentImage = sliderImages[currentSlide];

  return (
    <div
      ref={sliderRef}
      className={styles.heroSlider}
      style={{ background: currentImage.gradient }}
    >
      <div className={styles.heroOverlay}></div>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <Heading as="h1" ref={titleRef} className={styles.heroTitle}>
            {currentImage.title}
          </Heading>
          <p ref={subtitleRef} className={styles.heroSubtitle}>
            {currentImage.subtitle}
          </p>
          <div className={styles.heroButtons}>
            <Link className={styles.primaryButton} to="/docs/">
              Explore the Book
            </Link>
            <Link className={styles.secondaryButton} to="/docs/edge-kit">
              Edge Kit Guide
            </Link>
          </div>
        </div>
        <div ref={robotRef} className={styles.heroRobot}>
          <RobotSVG className={styles.robotSvg} />
        </div>
      </div>

      {/* Slider Dots */}
      <div className={styles.sliderDots}>
        {sliderImages.map((_, index) => (
          <button
            key={index}
            className={clsx(styles.dot, { [styles.activeDot]: index === currentSlide })}
            onClick={() => setCurrentSlide(index)}
            aria-label={`Go to slide ${index + 1}`}
          />
        ))}
      </div>

      {/* Decorative Elements */}
      <div className={styles.heroDecor}>
        <div className={styles.circle1}></div>
        <div className={styles.circle2}></div>
        <div className={styles.circle3}></div>
      </div>
    </div>
  );
}

function ModuleCard({ module, index }: { module: typeof bookModules[0]; index: number }) {
  const cardRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (typeof window !== 'undefined' && cardRef.current) {
      gsap.fromTo(
        cardRef.current,
        { opacity: 0, y: 60, scale: 0.9 },
        {
          opacity: 1,
          y: 0,
          scale: 1,
          duration: 0.6,
          delay: index * 0.1,
          ease: 'power3.out',
          scrollTrigger: {
            trigger: cardRef.current,
            start: 'top 85%',
            toggleActions: 'play none none none',
          },
        }
      );
    }
  }, [index]);

  return (
    <div ref={cardRef} className={styles.moduleCard}>
      <Link to={module.link} className={styles.moduleCardLink}>
        <div className={styles.moduleIcon} style={{ background: `linear-gradient(135deg, ${module.color} 0%, ${module.color}cc 100%)` }}>
          <span>{module.icon}</span>
        </div>
        <h3 className={styles.moduleTitle}>{module.title}</h3>
        <p className={styles.moduleDescription}>{module.description}</p>
        <div className={styles.moduleArrow}>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M5 12h14M12 5l7 7-7 7"/>
          </svg>
        </div>
      </Link>
    </div>
  );
}

function ModulesSection() {
  const sectionRef = useRef<HTMLElement>(null);
  const titleRef = useRef<HTMLHeadingElement>(null);

  useEffect(() => {
    if (typeof window !== 'undefined' && titleRef.current) {
      gsap.fromTo(
        titleRef.current,
        { opacity: 0, y: 40 },
        {
          opacity: 1,
          y: 0,
          duration: 0.8,
          ease: 'power3.out',
          scrollTrigger: {
            trigger: titleRef.current,
            start: 'top 80%',
            toggleActions: 'play none none none',
          },
        }
      );
    }
  }, []);

  return (
    <section ref={sectionRef} className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" ref={titleRef} className={styles.sectionTitle}>
          Book Modules
        </Heading>
        <p className={styles.sectionSubtitle}>
          Explore comprehensive content covering all aspects of Physical AI and Humanoid Robotics in education
        </p>
        <div className={styles.modulesGrid}>
          {bookModules.map((module, index) => (
            <ModuleCard key={module.title} module={module} index={index} />
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description="Comprehensive guide to Physical AI and Humanoid Robotics in Education - Learn robotics, AI ethics, and implementation strategies"
    >
      <HeroSlider />
      <main>
        <ModulesSection />
      </main>
    </Layout>
  );
}
